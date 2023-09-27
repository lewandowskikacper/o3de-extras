/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "JointsManipulationEditorComponent.h"
#include "AzCore/Debug/Trace.h"
#include "AzCore/base.h"
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
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>

namespace ROS2
{
    JointsManipulationEditorComponent::JointsManipulationEditorComponent()
    {
        m_jointStatePublisherConfiguration.m_topicConfiguration.m_type = "sensor_msgs::msg::JointState";
        m_jointStatePublisherConfiguration.m_topicConfiguration.m_topic = "joint_states";
        m_jointStatePublisherConfiguration.m_frequency = 25.0f;
    }


    AZ::JsonSerializationResult::Result JointJointInitialPositionSerializer::Load(
        void* outputValue,
        [[maybe_unused]] const AZ::Uuid& outputValueTypeId,
        const rapidjson::Value& inputValue,
        AZ::JsonDeserializerContext& context)
    {

        AZ_Info("JointsManipulationEditorComponent::JointJointInitialPositionSerializer", "Custom serializer is called");

        namespace JSR = AZ::JsonSerializationResult;

        auto configInstance = reinterpret_cast<JointsManipulationEditorComponent*>(outputValue);
        AZ_Assert(configInstance, "Output value for JointsManipulationEditorComponent can't be null.");

        JSR::ResultCode result(JSR::Tasks::ReadField);



        // auto copyIntoStruct = [&](const char* name, auto& dataRef)
        // {
        //     rapidjson::Value::ConstMemberIterator itr = inputValue.FindMember(name);
        //     if (itr != inputValue.MemberEnd() && itr->value.IsFloat())
        //     {
        //         dataRef = itr->value.GetFloat();
        //     }
        // };

        // copyIntoStruct("gnssOriginLatitude", configInstance->m_gnssConfiguration.m_originLatitudeDeg);
        // copyIntoStruct("gnssOriginLongitude", configInstance->m_gnssConfiguration.m_originLongitudeDeg);
        // copyIntoStruct("gnssOriginAltitude", configInstance->m_gnssConfiguration.m_originAltitude);


        rapidjson::Value::ConstMemberIterator itr = inputValue.FindMember("Initial positions");
        if (itr != inputValue.MemberEnd())
        {
            AZ::u32 index = 0;
            for(auto x = itr->value.MemberBegin(); x != itr->value.MemberEnd(); x++)
            {
            
                auto name =  x->name.GetString();
                auto position = x->value.GetFloat();
                AZ_Info("JointJointInitialPositionSerializer", "Found joint %s position %f",name, position);

                JointInitialPosition initial;
                initial.m_name = name;
                initial.m_position = position;
                initial.m_index = index;
                index++;

                configInstance->m_initialPositions.insert(initial);
            
            }

            AZ_Info("JointJointInitialPositionSerializer", "Collected %d joints", configInstance->m_initialPositions.size());


            // if(itr->value.IsArray())
            // {
            //     AZ_Info("JointJointInitialPositionSerializer", "found array");
            //     auto a = itr->value.GetArray();

            //     for (auto&x : a)
            //     {
            //         if (x.IsFloat())
            //         {
            //             auto f = x.GetFloat();
            //             AZ_Info("JointJointInitialPositionSerializer", "found float %f", f);

            //         }

            //         if(x.IsString())
            //         {
            //             auto f = x.GetString();
            //             AZ_Info("JointJointInitialPositionSerializer", "found string %s", f);
            //         }

            //     }

            //     // for ()
            //     {

            //     }

            // }
            
            // for (auto x = itr->value.Begin(); x != itr->value.End(); x++)
            // {
            //     if (x->IsFloat())
            //     {
            //         auto f = x->GetFloat();
            //         AZ_Info("JointJointInitialPositionSerializer", "found float %f", f);

            //     }

            //     if(x->IsString())
            //     {
            //         auto f = x->GetString();
            //         AZ_Info("JointJointInitialPositionSerializer", "found string %s", f);
            //     }

            //     if(x->IsArray())
            //     {
            //         // auto f = x->GetString();
            //         AZ_Info("JointJointInitialPositionSerializer", "found array");
            //     }

            // }

        }
        


        result.Combine(ContinueLoadingFromJsonObjectField(
            &configInstance->m_initialPositions,
            azrtti_typeid<decltype(configInstance->m_initialPositions)>(),
            inputValue,
            "Initial positions ordered",
            context));
        


        result.Combine(ContinueLoadingFromJsonObjectField(
            &configInstance->m_jointStatePublisherConfiguration,
            azrtti_typeid<decltype(configInstance->m_jointStatePublisherConfiguration)>(),
            inputValue,
            "JointStatePublisherConfiguration",
            context));

        result.Combine(ContinueLoadingFromJsonObjectField(
            &configInstance->m_positionCommandTopic,
            azrtti_typeid<decltype(configInstance->m_positionCommandTopic)>(),
            inputValue,
            "Position Command Topic",
            context));

        // result.Combine(ContinueLoadingFromJsonObjectField(
        //     &configInstance->m_initialPositions,
        //     azrtti_typeid<decltype(configInstance->m_initialPositions)>(),
        //     inputValue,
        //     "Initial positions",
        //     context));


        return context.Report(
            result,
            result.GetProcessing() != JSR::Processing::Halted ? "Successfully loaded GNSSSensorConfiguration information."
                                                              : "Failed to load GNSSSensorConfiguration information.");

    }

    AZ_CLASS_ALLOCATOR_IMPL(JointJointInitialPositionSerializer, AZ::SystemAllocator);

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


        if (auto jsonContext = azrtti_cast<AZ::JsonRegistrationContext*>(context))
        {
            jsonContext->Serializer<JointJointInitialPositionSerializer>()->HandlesType<JointsManipulationEditorComponent>();
        }

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
