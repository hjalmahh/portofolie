#ifndef AIS4104_APP_UI_STATE_SERIALIZER_H
#define AIS4104_APP_UI_STATE_SERIALIZER_H

#include "app/hardcoded_ui_states.h"

#include <visualization/imgui/windows/toolwindow.h>
#include <visualization/imgui/windows/posedisplaywindow.h>
#include <visualization/imgui/windows/robotcontrolwindows.h>
#include <visualization/imgui/windows/trajectoryloggerwindow.h>
#include <visualization/imgui/windows/worldobjectconfigurationwindow.h>

namespace AIS4104 {

//TASK: Implement functions or classes for serializing and deserializing window states. Use a file format such as XML or JSON.
// Use third party library from VCPKG, e.g., pugixml or nlohmann json.

inline Visualization::TrajectoryLoggerWindow::State load_logger_state(const std::filesystem::path &path)
{
    return default_logger_state();
}

inline void save_logger_state(const std::filesystem::path &path, const Visualization::TrajectoryLoggerWindow::State &state)
{
}

inline Visualization::ToolConfigurationWindow::State load_tool_state(const std::filesystem::path &path)
{
    return default_tool_state();
}

inline void save_tool_state(const std::filesystem::path &path, const Visualization::ToolConfigurationWindow::State &state)
{
}

inline Visualization::WorldObjectConfigurationWindow::State load_world_object_state(const std::filesystem::path &path)
{
    return default_world_object_state();
}

inline void save_world_object_state(const std::filesystem::path &path, const Visualization::WorldObjectConfigurationWindow::State &state)
{
}

}

#endif
