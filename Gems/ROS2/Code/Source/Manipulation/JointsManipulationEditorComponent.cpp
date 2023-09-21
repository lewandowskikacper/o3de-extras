/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "JointsManipulationEditorComponent.h"
#include "AzCore/Debug/Trace.h"
#include "AzCore/std/containers/unordered_map.h"
#include "AzCore/std/containers/vector.h"
#include "AzCore/std/string/string.h"
#include "JointsManipulationComponent.h"
#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/Manipulation/Controllers/JointsPositionControllerRequests.h>
#include <ROS2/Manipulation/JointInfo.h>
#include <ROS2/ROS2GemUtilities.h>
#include <ROS2/Utilities/ROS2Names.h>
#include <Source/ArticulationLinkComponent.h>
#include <Source/HingeJointComponent.h>
#include <AzCore/Serialization/DataPatch.h>
#include <AzCore/Serialization/ObjectStream.h>
#include <AzCore/Serialization/Utils.h>
#include <AzCore/Asset/AssetSerializer.h>
#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/IO/ByteContainerStream.h>
#include <AzCore/IO/SystemFile.h>
#include <AzCore/Serialization/DataPatch.h>
#include <AzCore/Serialization/ObjectStream.h>
#include <AzCore/Serialization/Utils.h>

namespace ROS2
{
    JointsManipulationEditorComponent::JointsManipulationEditorComponent()
    {
        m_jointStatePublisherConfiguration.m_topicConfiguration.m_type = "sensor_msgs::msg::JointState";
        m_jointStatePublisherConfiguration.m_topicConfiguration.m_topic = "joint_states";
        m_jointStatePublisherConfiguration.m_frequency = 25.0f;
    }

    void JointsManipulationEditorComponent::BuildGameEntity(AZ::Entity* gameEntity)
    {
        AZStd::vector<AZStd::string> jointOrderedNames;
        AZStd::unordered_map<AZStd::string, float> initialPositions;

        // for (auto &[index, initial]: m_initialPositions)
        // {
        //     auto &[name, position] = initial;
        //     jointOrderedNames.push_back(name);
        //     initialPositions[name] = position;
        // }


        // for (auto &[name, initial]: m_initialPositions)
        // {
        //     auto &[ position, index] = initial; // TODO sort by index
        //     jointOrderedNames.push_back(name);
        //     initialPositions[name] = position;
        // }

        for (auto &[name, position, index]: m_initialPositions)
        {
            // auto &[ position, index] = initial; // TODO sort by index
            AZ_Info("JointsManipulationEditorComponent", "another joint %s at index %u, position: %f", name.c_str(), index, position);
            jointOrderedNames.push_back(name);
            initialPositions[name] = position;
        }

        gameEntity->CreateComponent<JointsManipulationComponent>(
            m_jointStatePublisherConfiguration, initialPositions, jointOrderedNames, m_positionCommandTopic);
    }

    void JointsManipulationEditorComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("ROS2Frame"));
        required.push_back(AZ_CRC_CE("JointsControllerService"));
    }

    void JointsManipulationEditorComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("JointsManipulationService"));
    }

    void JointsManipulationEditorComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("JointsManipulationService"));
    }

    bool JointsManipulationEditorComponent::ConvertVersion(AZ::SerializeContext& context, AZ::SerializeContext::DataElementNode& classElement)
    {
        // based on EditorMaterialComponentSlot::ConvertVersion

        AZ_Info("JointsManipulationEditorComponent::ConvertVersion", "Check if conversion is needed.");

        AZ_Error("JointsManipulationEditorComponent::ConvertVersion", false, "Enter Converter Conversion")

        if (classElement.GetVersion() < 1)
        {
            AZ_Info("JointsManipulationEditorComponent::ConvertVersion", "Converting versions.");

            constexpr AZ::u32 initialPositionsCrc = AZ_CRC_CE("Initial positions");
            AZStd::unordered_map<AZStd::string, float> oldInitialPositions;
            if (!classElement.GetChildData(initialPositionsCrc, oldInitialPositions))
            {
                AZ_Error("JointsManipulationEditorComponent::ConvertVersion", false, "Failed to get initialPositions element");
                return false;
            }

            if (!classElement.RemoveElementByName(initialPositionsCrc))
            {
                AZ_Error("JointsManipulationEditorComponent::ConvertVersion", false, "Failed to remove initialPositions element");
                return false;
            }

            AZStd::map<AZ::u32, AZStd::pair<AZStd::string, float>> newInitialPositions;
            AZ::u32 i = 0;
            for(auto [name, position]: oldInitialPositions)
            {
                newInitialPositions[i] = {name, position};
                i++;
            }

            if(classElement.AddElementWithData(context, "Initial positions", newInitialPositions) == -1)
            {
                AZ_Error("JointsManipulationEditorComponent::ConvertVersion", false, "Failed to add initialPositions element");
                return false;
            }
        }

        return true;
    }

     AZStd::map<AZ::u32, AZStd::pair<AZStd::string, float>> convertPositions(AZStd::unordered_map<AZStd::string, float> oldInitialPositions)
     {
        AZStd::map<AZ::u32, AZStd::pair<AZStd::string, float>> newInitialPositions;
        AZ::u32 i = 0;
        for(auto [name, position]: oldInitialPositions)
        {
            newInitialPositions[i] = {name, position};
            i++;
        }
        return newInitialPositions;
     }

    void JointsManipulationEditorComponent::Reflect(AZ::ReflectContext* context)
    {
        JointInitialPosition::Reflect(context);
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<JointsManipulationEditorComponent, AZ::Component>()
                ->Version(1)
                // ->Version(1, &JointsManipulationEditorComponent::ConvertVersion)
                ->Field("JointStatePublisherConfiguration", &JointsManipulationEditorComponent::m_jointStatePublisherConfiguration)
                ->Field("Initial positions ordered", &JointsManipulationEditorComponent::m_initialPositions)
                // ->TypeChange<AZStd::unordered_map<AZStd::string, float>, AZStd::map<AZ::u32, AZStd::pair<AZStd::string, float>>>("Initial positions", 0, 1, 
                    // AZStd::function<AZStd::map<AZ::u32, AZStd::pair<AZStd::string, float>>(AZStd::unordered_map<AZStd::string, float>)>(convertPositions))
                ->Field("Position Command Topic", &JointsManipulationEditorComponent::m_positionCommandTopic);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<JointsManipulationEditorComponent>("JointsManipulationEditorComponent", "Component for manipulation of joints")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &JointsManipulationEditorComponent::m_jointStatePublisherConfiguration,
                        "Joint State Publisher",
                        "Configuration of Joint State Publisher")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &JointsManipulationEditorComponent::m_initialPositions,
                        "Initial positions",
                        "Initial positions of all the joints. Position Controller will forward control messages to joints in the order they appear here.")
                    ->Attribute(AZ::Edit::Attributes::ContainerReorderAllow, true)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &JointsManipulationEditorComponent::m_positionCommandTopic,
                        "Position Conmmand Topic",
                        "Topic on which position commands are received");
            }
        }
    }
} // namespace ROS2
