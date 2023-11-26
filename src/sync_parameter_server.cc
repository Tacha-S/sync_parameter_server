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

#include "sync_parameter_server/sync_parameter_server.hh"
#include <rcl_interfaces/msg/parameter.hpp>

namespace sync_parameter_server
{

ParameterInfo::ParameterInfo(const std::string& node, const std::string& param,
                             const std::shared_ptr<rclcpp::AsyncParametersClient>& client)
  : node_(node), param_(param), client_(client)
{
}

std::string ParameterInfo::node() const
{
  return node_;
}

std::string ParameterInfo::param() const
{
  return param_;
}

std::shared_ptr<rclcpp::AsyncParametersClient> ParameterInfo::client() const
{
  return client_;
}

rcl_interfaces::msg::ParameterValue ParameterInfo::value() const
{
  return value_;
}

void ParameterInfo::setValue(const rcl_interfaces::msg::ParameterValue& value)
{
  value_ = value;
}

SyncParameterServer::SyncParameterServer(const rclcpp::NodeOptions& options)
  : rclcpp::Node("sync_parameter_server", options)
{
  param_event_handler_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
  this->declare_parameter("sync_parameters", std::vector<std::string>());
  updateSyncSetting();
  parameter_callback_handle_ =
      this->add_on_set_parameters_callback(std::bind(&SyncParameterServer::setParameters, this, std::placeholders::_1));
}

SyncParameterServer::~SyncParameterServer()
{
}

void SyncParameterServer::declareDefaultParameter(const rclcpp::Parameter& type, const std::string& param)
{
  if (type.as_string() == "bool")
  {
    this->declare_parameter(param + ".default", rclcpp::ParameterType::PARAMETER_BOOL);
  }
  else if (type.as_string() == "int")
  {
    this->declare_parameter(param + ".default", rclcpp::ParameterType::PARAMETER_INTEGER);
  }
  else if (type.as_string() == "double")
  {
    this->declare_parameter(param + ".default", rclcpp::ParameterType::PARAMETER_DOUBLE);
  }
  else if (type.as_string() == "string")
  {
    this->declare_parameter(param + ".default", rclcpp::ParameterType::PARAMETER_STRING);
  }
  else if (type.as_string() == "byte_array")
  {
    this->declare_parameter(param + ".default", rclcpp::ParameterType::PARAMETER_BYTE_ARRAY);
  }
  else if (type.as_string() == "bool_array")
  {
    this->declare_parameter(param + ".default", rclcpp::ParameterType::PARAMETER_BOOL_ARRAY);
  }
  else if (type.as_string() == "integer_array")
  {
    this->declare_parameter(param + ".default", rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY);
  }
  else if (type.as_string() == "double_array")
  {
    this->declare_parameter(param + ".default", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
  }
  else if (type.as_string() == "string_array")
  {
    this->declare_parameter(param + ".default", rclcpp::ParameterType::PARAMETER_STRING_ARRAY);
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Parameter type %s is not supported", type.as_string().c_str());
  }
}

rcl_interfaces::msg::SetParametersResult
SyncParameterServer::setParameters(const std::vector<rclcpp::Parameter>& parameters)
{
  rcl_interfaces::msg::SetParametersResult ret;

  updateSyncSetting();
  for (const auto& param : parameters)
  {
    auto div = param.get_name().rfind('.');
    if (param.get_name().substr(div + 1, param.get_name().size()) == "default")
    {
      auto sync_name = param.get_name().substr(0, div);
      auto sync_param_paths = sync_parameters_[sync_name];

      if (sync_param_paths.first != param.get_type())
      {
        ret.successful = false;
        ret.reason = "Parameter type is different";
        return ret;
      }

      for (auto& param_info : sync_param_paths.second)
      {
        auto value = param.get_value_message();
        if (value.type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE ||
            value.type == rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
        {
          double scale, offset;
          this->get_parameter(sync_name + "." + param_info->node() + "/" + param_info->param() + ".scale", scale);
          this->get_parameter(sync_name + "." + param_info->node() + "/" + param_info->param() + ".offset", offset);
          value.integer_value = static_cast<int>(std::round(scale * static_cast<double>(value.integer_value) + offset));
          value.double_value = scale * value.double_value + offset;
        }
        if (param_info->value() != value)
        {
          auto m = rclcpp::Parameter(param_info->param(), value);
          param_info->setValue(value);
          auto future = param_info->client()->set_parameters({ m });
        }
      }
    }
  }
  ret.successful = true;
  return ret;
}

void SyncParameterServer::updateParameterCallback(const std::string& sync_name, const std::string& node_name,
                                                  const rclcpp::Parameter& param)
{
  RCLCPP_INFO(this->get_logger(), "Parameter %s on %s is updated", param.get_name().c_str(), node_name.c_str());
  auto sync_param_paths = sync_parameters_.find(sync_name);
  bool updated = false;
  auto base_value = param.get_value_message();

  // if (sync_param_paths != sync_parameters_.end())
  {
    for (const auto& param_info : sync_param_paths->second.second)
    {
      if (node_name == param_info->node())
      {
        if (param_info->value() == param.get_value_message())
          updated = true;
        param_info->setValue(param.get_value_message());
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE ||
            param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
        {
          double scale, offset;
          this->get_parameter(sync_name + "." + param_info->node() + "/" + param_info->param() + ".scale", scale);
          this->get_parameter(sync_name + "." + param_info->node() + "/" + param_info->param() + ".offset", offset);
          if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
          {
            base_value.integer_value =
                static_cast<int>(std::round((static_cast<double>(param.as_int()) - offset) / scale));
          }
          else
          {
            base_value.double_value = (param.as_double() - offset) / scale;
          }
        }
      }
    }

    if (!updated)
    {
      for (auto& param_info : sync_param_paths->second.second)
      {
        if (node_name != param_info->node())
        {
          auto value = base_value;
          if (value.type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE ||
              value.type == rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
          {
            double scale, offset;
            this->get_parameter(sync_name + "." + param_info->node() + "/" + param_info->param() + ".scale", scale);
            this->get_parameter(sync_name + "." + param_info->node() + "/" + param_info->param() + ".offset", offset);
            value.integer_value =
                static_cast<int>(std::round(scale * static_cast<double>(value.integer_value) + offset));
            value.double_value = scale * value.double_value + offset;
          }
          if (param_info->value() != value)
          {
            auto m = rclcpp::Parameter(param_info->param(), value);
            param_info->setValue(value);
            auto future = param_info->client()->set_parameters({ m });
          }
        }
      }
    }
  }
}

void SyncParameterServer::updateSyncSetting()
{
  std::vector<std::string> sync_names;
  this->get_parameter("sync_parameters", sync_names);

  for (const auto& sync_name : sync_names)
  {
    if (sync_parameters_.find(sync_name) == sync_parameters_.end())
    {
      this->declare_parameter(sync_name + ".params", std::vector<std::string>());
      this->declare_parameter(sync_name + ".type", rclcpp::ParameterType::PARAMETER_STRING);
      rclcpp::Parameter type_param;
      this->get_parameter(sync_name + ".type", type_param);
      declareDefaultParameter(type_param, sync_name);
      rclcpp::Parameter default_value;
      this->get_parameter(sync_name + ".default", default_value);
      sync_parameters_.emplace(sync_name,
                               std::make_pair(default_value.get_type(), std::vector<std::shared_ptr<ParameterInfo>>()));
    }
    std::vector<std::string> sync_param_paths;
    this->get_parameter(sync_name + ".params", sync_param_paths);
    rclcpp::Parameter type_param;
    this->get_parameter(sync_name + ".type", type_param);
    rclcpp::Parameter default_value;
    this->get_parameter(sync_name + ".default", default_value);
    for (const auto& sync_param_path : sync_param_paths)
    {
      auto div = sync_param_path.rfind("/");
      auto node_name = sync_param_path.substr(0, div);
      auto param_name = sync_param_path.substr(div + 1, sync_param_path.size());
      if (std::find_if(sync_parameters_[sync_name].second.begin(), sync_parameters_[sync_name].second.end(),
                       [node_name, param_name](auto e) {
                         return e->node() == node_name && e->param() == param_name;
                       }) == sync_parameters_[sync_name].second.end())
      {
        auto client = std::make_shared<rclcpp::AsyncParametersClient>(this, node_name);
        if (default_value.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE ||
            default_value.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
        {
          double scale, offset;
          this->declare_parameter(sync_name + "." + sync_param_path + ".scale", 1.0);
          this->declare_parameter(sync_name + "." + sync_param_path + ".offset", 0.0);
          this->get_parameter(sync_name + "." + sync_param_path + ".scale", scale);
          this->get_parameter(sync_name + "." + sync_param_path + ".offset", offset);
          rclcpp::Parameter p;
          if (default_value.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
          {
            p = rclcpp::Parameter(param_name, rclcpp::ParameterValue(static_cast<int>(
                                                  std::round(scale * default_value.as_int() + offset))));
          }
          else
          {
            p = rclcpp::Parameter(param_name, rclcpp::ParameterValue(scale * default_value.as_double() + offset));
          }
          client->set_parameters({ p });
        }
        else
        {
          client->set_parameters({ default_value });
        }
        sync_parameters_[sync_name].second.emplace_back(std::make_shared<ParameterInfo>(node_name, param_name, client));
        cb_handles_.push_back(param_event_handler_->add_parameter_callback(
            param_name,
            std::bind(&SyncParameterServer::updateParameterCallback, this, sync_name, node_name, std::placeholders::_1),
            node_name));
        auto param_info = sync_parameters_[sync_name].second.back();
        auto future = param_info->client()->get_parameters(
            { param_name }, [this, sync_name, param_name,
                             param_info](const std::shared_future<std::vector<rclcpp::Parameter>>& response) {
              if (sync_parameters_[sync_name].first != response.get()[0].get_type())
              {
                RCLCPP_ERROR(this->get_logger(), "Parameter %s has different type between nodes", param_name.c_str());
                rclcpp::shutdown();
                exit(1);
              }
            });
        RCLCPP_INFO(this->get_logger(), "Register parameter %s on %s", param_name.c_str(), node_name.c_str());
      }
    }
  }
}

}  // namespace sync_parameter_server

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sync_parameter_server::SyncParameterServer>());
  rclcpp::shutdown();
  return 0;
}
