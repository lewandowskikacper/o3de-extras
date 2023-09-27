/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include "AzCore/base.h"
#include "ROS2/Manipulation/JointInfo.h"
#include <AzCore/std/containers/unordered_map.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/string/string.h>
#include <AzToolsFramework/ToolsComponents/EditorComponentBase.h>
#include <ROS2/Communication/PublisherConfiguration.h>
#include <AzCore/Serialization/Json/BaseJsonSerializer.h>
#include <AzCore/Serialization/Json/RegistrationContext.h>

namespace ROS2
{

    class JointJointInitialPositionSerializer : public AZ::BaseJsonSerializer
    {
    public:
        AZ_RTTI(ROS2::JointJointInitialPositionSerializer, "{3c382164-54a3-4901-8835-14af9ff9c58b}", AZ::BaseJsonSerializer);
        AZ_CLASS_ALLOCATOR_DECL;

        AZ::JsonSerializationResult::Result Load(
            void* outputValue,
            const AZ::Uuid& outputValueTypeId,
            const rapidjson::Value& inputValue,
            AZ::JsonDeserializerContext& context) override;
    };

    //! Editor Component responsible for a hierarchical system of joints such as robotic arm with Articulations or Hinge Joints.
    class JointsManipulationEditorComponent : public AzToolsFramework::Components::EditorComponentBase
    {
        friend class JointJointInitialPositionSerializer;
    public:
        JointsManipulationEditorComponent();
        ~JointsManipulationEditorComponent() = default;
        AZ_EDITOR_COMPONENT(JointsManipulationEditorComponent, "{BF2F77FD-92FB-4730-92C7-DDEE54F508BF}");

        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void Reflect(AZ::ReflectContext* context);

        // AzToolsFramework::Components::EditorComponentBase overrides
        void BuildGameEntity(AZ::Entity* gameEntity) override;

        static bool ConvertVersion(AZ::SerializeContext& context, AZ::SerializeContext::DataElementNode& classElement);

    private:
        PublisherConfiguration m_jointStatePublisherConfiguration;

        AZStd::set<JointInitialPosition, JointInitiaPositionComparator> m_initialPositions;
        // AZStd::unordered_map<AZStd::string, AZStd::pair<float, AZ::u32>> m_initialPositions;
        // AZStd::map<AZ::u32, AZStd::pair<AZStd::string, float>> m_initialPositions;
        AZStd::string m_positionCommandTopic = "/position_controller/commands";
    };
} // namespace ROS2
