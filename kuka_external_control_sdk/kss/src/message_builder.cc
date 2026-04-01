// Copyright 2023 KUKA Deutschland GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "kuka/external-control-sdk/kss/message_builder.h"
#include <tinyxml2.h> // for parsing xml files 
#include <iostream>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <stdexcept>
#include <string>

#include <fstream>  

using namespace tinyxml2;


std::vector<JointInfo> LoadJointsFromRSIConfig(const std::string& filename) {
    std::vector<JointInfo> joints;
    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(filename.c_str()) != tinyxml2::XML_SUCCESS) return joints;
    tinyxml2::XMLElement* root = doc.FirstChildElement("ROOT");
    if (!root) return joints;
    tinyxml2::XMLElement* receive = root->FirstChildElement("RECEIVE");
    if (!receive) return joints;
    tinyxml2::XMLElement* elements = receive->FirstChildElement("ELEMENTS");
    if (!elements) return joints;

    for (tinyxml2::XMLElement* elem = elements->FirstChildElement("ELEMENT");
         elem; elem = elem->NextSiblingElement("ELEMENT")) {
        const char* tag = elem->Attribute("TAG");
        const char* type = elem->Attribute("TYPE");
        int index = elem->IntAttribute("INDX", 0);
        // Accept both AK.A and AK.E (rotary and external/linear axes)
        if (tag && type && (std::string(tag).rfind("AK.A", 0) == 0 || std::string(tag).rfind("AK.E", 0) == 0)) {
            std::string tagstr(tag);
            bool is_linear = (tagstr.find("E") != std::string::npos);
            JointInfo joint{tagstr.substr(tagstr.find('.')+1), tag, type, index, is_linear};
            joints.push_back(joint);
        }
    }
    return joints;
}

namespace kuka::external::control::kss {

uint8_t MotionState::file_saved = 0; 
uint8_t ControlSignal::file_saved = 0; 

void MotionState::CreateFromXML(const char *incoming_xml) {
    if (!incoming_xml) throw std::invalid_argument("Received XML is null");

    // Optional: Save received XML once for debugging
    if (file_saved == 0){
        std::ofstream xml_out("/mnt/nova_ssd/workspaces/isaac_ros-dev/received_motionstate.xml", std::ios::out | std::ios::trunc);
        if (xml_out.is_open()) {
            xml_out << incoming_xml;
            xml_out.close();
        }
        file_saved = 1;
    }

    using namespace tinyxml2;
    XMLDocument doc;
    if (doc.Parse(incoming_xml) != XML_SUCCESS)
        throw std::runtime_error("Failed to parse XML");

    // Top-level <Rob>
    XMLElement* robElem = doc.FirstChildElement("Rob");
    if (!robElem) throw std::runtime_error("No <Rob> element");

    // --- 1. Cartesian positions <RIst .../> ---
    XMLElement* ristElem = robElem->FirstChildElement("RIst");
    if (!ristElem) throw std::runtime_error("No <RIst> element");
    static const char* cartesianNames[6] = {"X", "Y", "Z", "A", "B", "C"};
    for (int i = 0; i < 6; ++i) {
        double val = std::numeric_limits<double>::quiet_NaN();
        XMLError err = ristElem->QueryDoubleAttribute(cartesianNames[i], &val);
        if (err != XML_SUCCESS)
            throw std::runtime_error(std::string("Missing or invalid attribute in <RIst>: ") + cartesianNames[i]);
        // For A, B, C (indices 3,4,5): convert deg->rad
        if (i > 2) val *= (M_PI / 180.0);
        measured_cartesian_positions_[i] = val;
    }

    // --- 2. Joint positions <AIPos .../> ---
    XMLElement* aiPosElem = robElem->FirstChildElement("AIPos");
    XMLElement* eiPosElem = robElem->FirstChildElement("EIPos");
    if (!aiPosElem) throw std::runtime_error("No <AIPos> element");
    if (!eiPosElem) throw std::runtime_error("No <EIPos> element");

    for (size_t i = 0; i < joint_info_->size(); ++i) {
        const auto& joint = (*joint_info_)[i];
        double jointValue = std::numeric_limits<double>::quiet_NaN();
        if (joint.is_linear) {
            // External axis from EIPos, convert mm to m
            if (eiPosElem->QueryDoubleAttribute(joint.name.c_str(), &jointValue) != XML_SUCCESS)
                throw std::runtime_error("Missing EIPos joint attribute: " + joint.name);
            measured_positions_[i] = jointValue / 1000.0;
        } else {
            // Rotary axis from AIPos, convert deg to rad
            if (aiPosElem->QueryDoubleAttribute(joint.name.c_str(), &jointValue) != XML_SUCCESS)
                throw std::runtime_error("Missing AIPos joint attribute: " + joint.name);
            measured_positions_[i] = jointValue * (M_PI / 180.0);
        }
    }


    // --- 4. IPOC <IPOC>value</IPOC> ---
    XMLElement* ipocElem = robElem->FirstChildElement("IPOC");
    ipoc_ = 0;
    if (ipocElem && ipocElem->GetText())
        ipoc_ = std::stol(ipocElem->GetText());

    // --- Mark state as valid ---
    has_positions_ = true;
    has_cartesian_positions_ = true;
}

void ControlSignal::AppendToXMLString(std::string_view str) {
  strncat(xml_string_, str.data(),
          kBufferSize - strnlen(xml_string_, kBufferSize) - 1);
}

std::optional<std::string_view> ControlSignal::CreateXMLString(int last_ipoc, bool stop_control) {
    static std::string xml_output; // buffer that will be returned as a view

    XMLDocument doc;
    // <Sen Type="MAXWELL">
    XMLElement* senElem = doc.NewElement("Sen");
    senElem->SetAttribute("Type", "MAXWELL");
    doc.InsertFirstChild(senElem);

    // <AK .../> for joint positions
    XMLElement* akElem = doc.NewElement("AK");
    for (size_t i = 0; i <  joint_info_->size(); ++i) {
        const auto& joint = (*joint_info_)[i];
        double correction = joint_position_values_[i] - initial_positions_[i];
        if (joint.is_linear) {
            double correction_mm = correction * 1000.0; // meters -> mm
            akElem->SetAttribute(joint.name.c_str(), correction_mm);
        } else {
            double correction_deg = correction * (180.0 / M_PI);
            akElem->SetAttribute(joint.name.c_str(), correction_deg);
        }
    }
    senElem->InsertEndChild(akElem);

    // <Stop>...</Stop>
    XMLElement* stopElem = doc.NewElement("Stop");
    stopElem->SetText(stop_control ? "1" : "0");
    senElem->InsertEndChild(stopElem);

    // <IPOC>...</IPOC>
    XMLElement* ipocElem = doc.NewElement("IPOC");
    ipocElem->SetText(std::to_string(last_ipoc).c_str());
    senElem->InsertEndChild(ipocElem);

    // Convert to string
    XMLPrinter printer;
    doc.Print(&printer);

    xml_output = printer.CStr();

    if (file_saved == 0){
      std::ofstream xml_out("/mnt/nova_ssd/workspaces/isaac_ros-dev/sent_state.xml", std::ios::out | std::ios::trunc);
      if (xml_out.is_open()) {
          xml_out << xml_output;
          xml_out.close();
      }
      file_saved = 1;
  }
    return xml_output;
}

void ControlSignal::SetInitialPositions(const MotionState &initial_positions) {
  has_initial_positions_ = true;
  std::copy(initial_positions.GetMeasuredPositions().cbegin(),
            initial_positions.GetMeasuredPositions().cend(),
            initial_positions_.begin());
}

} // namespace kuka::external::control::kss
