/*********************************************************************
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ********************************************************************/

#ifndef SYNC_PARAMETER_SERVER_SYNC_PARAMETER_SERVER_HH
#define SYNC_PARAMETER_SERVER_SYNC_PARAMETER_SERVER_HH

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rcl_interfaces/msg/parameter_event.hpp>

namespace sync_parameter_server
{

class ParameterInfo
{
public:
  ParameterInfo(const std::string& node, const std::string& param,
                const std::shared_ptr<rclcpp::AsyncParametersClient>& client);

  std::string node() const;
  std::string param() const;
  std::shared_ptr<rclcpp::AsyncParametersClient> client() const;
  rcl_interfaces::msg::ParameterValue value() const;
  void setValue(const rcl_interfaces::msg::ParameterValue& value);

private:
  std::string node_;
  std::string param_;
  std::shared_ptr<rclcpp::AsyncParametersClient> client_;
  rcl_interfaces::msg::ParameterValue value_;
};

class SyncParameterServer : public rclcpp::Node
{
public:
  explicit SyncParameterServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~SyncParameterServer() override;
  rcl_interfaces::msg::SetParametersResult setParameters(const std::vector<rclcpp::Parameter>& parameters);
  void updateParameterCallback(const std::string& sync_name, const std::string& node_name,
                               const rclcpp::Parameter& param);
  void updateSyncSetting();

private:
  void declareDefaultParameter(const rclcpp::Parameter& type, const std::string& param);

  std::unordered_map<std::string, std::pair<rclcpp::ParameterType, std::vector<std::shared_ptr<ParameterInfo>>>>
      sync_parameters_;

  std::shared_ptr<rclcpp::ParameterEventHandler> param_event_handler_;
  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
  std::vector<std::shared_ptr<rclcpp::ParameterCallbackHandle>> cb_handles_;
};
}  // namespace sync_parameter_server

#endif  // SYNC_PARAMETER_SERVER_SYNC_PARAMETER_SERVER_HH
