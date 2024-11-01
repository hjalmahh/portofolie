#ifndef VISUALIZATION_ROBOTSCENE_H
#define VISUALIZATION_ROBOTSCENE_H

#include "visualization/imgui/imguiwindowcontext.h"

#include <threepp/threepp.hpp>

#include <threepp/extras/imgui/ImguiContext.hpp>

#include <threepp/objects/Robot.hpp>

namespace AIS4104::Visualization {

class RobotScene
{
public:
    RobotScene(std::shared_ptr<threepp::Robot> robot, std::vector<std::shared_ptr<ImguiWindow>> imgui_items);

    ~RobotScene();

    void set_tool(std::shared_ptr<threepp::Object3D> tool);
    void set_tool(std::shared_ptr<threepp::Object3D> tool, threepp::Matrix4 tool_tf);
    void set_tool_transform(threepp::Matrix4 tool_tf);

    void reset_tool();

    void set_world_object(std::shared_ptr<threepp::Object3D> tool);
    void clear_world_object();

    threepp::Robot& robot();
    const threepp::Robot& robot() const;

    void render() const;

private:
    threepp::Matrix4 m_tool_tf;
    std::shared_ptr<ImguiWindowContext> m_imgui;
    std::shared_ptr<threepp::Robot> m_robot;
    std::shared_ptr<threepp::Scene> m_scene;
    std::unique_ptr<threepp::Canvas> m_canvas;
    std::shared_ptr<threepp::Object3D> m_tool;
    std::shared_ptr<threepp::Object3D> m_world_object;
    std::unique_ptr<threepp::IOCapture> m_capture;
    std::unique_ptr<threepp::GLRenderer> m_renderer;
    std::shared_ptr<threepp::OrbitControls> m_controls;
    std::shared_ptr<threepp::PerspectiveCamera> m_camera;

    void setup_scene();
    void add_camera();
    void add_scene_grid();
    void add_scene_objects();
};

}

#endif
