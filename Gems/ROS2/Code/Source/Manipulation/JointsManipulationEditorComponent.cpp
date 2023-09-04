/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "JointsManipulationEditorComponent.h"
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
        gameEntity->CreateComponent<JointsManipulationComponent>(
            m_jointStatePublisherConfiguration, m_initialPositions, m_jointOrderedNames, m_positionCommandTopic);
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

    void JointsManipulationEditorComponent::Activate()
    {
        if (m_jointOrderedNames.empty())
        {
            for (auto [jointName, jointInfo] : m_initialPositions)
            {
                m_jointOrderedNames.push_back(jointName);
            }
        }
    }

    void JointsManipulationEditorComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<JointsManipulationEditorComponent, AZ::Component>()
                ->Version(0)
                ->Field("JointStatePublisherConfiguration", &JointsManipulationEditorComponent::m_jointStatePublisherConfiguration)
                ->Field("Initial positions", &JointsManipulationEditorComponent::m_initialPositions)
                ->Field("Ordered Joint Names", &JointsManipulationEditorComponent::m_jointOrderedNames)
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
                        "Initial positions of all the joints")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &JointsManipulationEditorComponent::m_jointOrderedNames,
                        "Ordered Joint Names",
                        "Position Controller will forward control messages to joints in the order the appear here.")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &JointsManipulationEditorComponent::m_positionCommandTopic,
                        "Position Conmmand Topic",
                        "Topic on which position commands are received");
            }
        }
    }
} // namespace ROS2
