#include <utility>

#include "visualization/robotscene.h"

using namespace AIS4104::Visualization;

namespace tpp = threepp;

RobotScene::RobotScene(std::shared_ptr<threepp::Robot> robot, std::vector<std::shared_ptr<ImguiWindow>> imgui_items)
    : m_robot(std::move(robot))
{
    std::unordered_map<std::string, tpp::Canvas::ParameterValue> params{{"aa", 4}};
    m_canvas = std::make_unique<tpp::Canvas>("Robot renderer", params);
    m_imgui = std::make_shared<ImguiWindowContext>(*m_canvas, std::move(imgui_items));

    m_renderer = std::make_unique<tpp::GLRenderer>(m_canvas->size());
    m_renderer->setClearColor(tpp::Color::aliceblue);

    setup_scene();

    m_capture = std::make_unique<tpp::IOCapture>();
    m_capture->preventMouseEvent = []
    {
        return ImGui::GetIO().WantCaptureMouse;
    };
    m_canvas->setIOCapture(m_capture.get());
    m_canvas->onWindowResize([&](tpp::WindowSize size)
        {
            m_camera->aspect = size.aspect();
            m_camera->updateProjectionMatrix();
            m_renderer->setSize(size);
        }
    );
}

RobotScene::~RobotScene()
{
    if(m_canvas->isOpen())
        m_canvas->close();
    m_imgui.reset();
}

void RobotScene::set_tool(std::shared_ptr<threepp::Object3D> tool)
{
    threepp::Matrix4 i;
    set_tool(std::move(tool), i.identity());
}

void RobotScene::set_tool(std::shared_ptr<threepp::Object3D> tool, threepp::Matrix4 tool_tf)
{
    reset_tool();
    m_tool = std::move(tool);
    m_scene->add(m_tool);
    m_tool_tf = tool_tf;
}

void RobotScene::set_tool_transform(threepp::Matrix4 tool_tf)
{
    m_tool_tf = tool_tf;
}

void RobotScene::reset_tool()
{
    if(m_tool)
    {
        m_scene->remove(*m_tool);
        m_tool.reset();
        m_tool_tf.identity();
    }
}

void RobotScene::set_world_object(std::shared_ptr<threepp::Object3D> world_object)
{
    clear_world_object();
    m_world_object = std::move(world_object);
    m_scene->add(m_world_object);
}

void RobotScene::clear_world_object()
{
    if(m_world_object)
    {
        m_scene->remove(*m_world_object);
        m_world_object.reset();
    }
}

threepp::Robot& RobotScene::robot()
{
    return *m_robot;
}

const threepp::Robot& RobotScene::robot() const
{
    return *m_robot;
}

void RobotScene::render() const
{
    m_imgui->initialize();
    tpp::Clock clock;
    m_canvas->animate([&]()
        {
            if(m_tool != nullptr)
            {
                tpp::Matrix4 t_sb = m_robot->computeEndEffectorTransform(m_robot->jointValuesWithConversionFromRadiansToDeg(), true);
                m_tool->position.setFromMatrixPosition(t_sb);
                m_tool->quaternion.setFromRotationMatrix(t_sb.multiply(m_tool_tf));
            }
            m_renderer->render(*m_scene, *m_camera);
            m_imgui->render();
        }
    );
}

void RobotScene::setup_scene()
{
    m_scene = tpp::Scene::create();
    add_scene_grid();
    add_camera();
    add_scene_objects();
}

void RobotScene::add_scene_grid()
{
    float size = 10.f;
    auto material = tpp::ShadowMaterial::create();
    auto plane = tpp::Mesh::create(tpp::PlaneGeometry::create(size, size), material);
    plane->rotation.y = -tpp::math::PI / 2.f;
    plane->receiveShadow = true;

    auto grid = tpp::GridHelper::create(size, size, tpp::Color::yellowgreen);
    grid->rotation.y = tpp::math::PI / 2.f;
    plane->add(grid);
    m_scene->add(grid);
}

void RobotScene::add_camera()
{
    m_camera = tpp::PerspectiveCamera::create(75.f, m_canvas->aspect(), 0.1f, 100.f);
    m_camera->position.set(0.f, 1.25f, 1.f);
    m_controls = std::make_unique<tpp::OrbitControls>(*m_camera, *m_canvas);
}

void RobotScene::add_scene_objects()
{
    auto light = tpp::HemisphereLight::create(tpp::Color::aliceblue, tpp::Color::grey);
    m_scene->add(light);
    m_scene->add(*m_robot);
    // m_scene->add(*m_tool);
}
