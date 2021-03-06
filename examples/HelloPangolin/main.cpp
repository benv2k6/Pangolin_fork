#include <imgui.h>
#include <pangolin/pangolin.h>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <chrono>

#include <imgui_impl_opengl3.h>
#include <imgui_impl_win32.h>

// Converts degrees to radians.
#define degreesToRadians(angleDegrees) (angleDegrees * M_PI / 180.0)

// Converts radians to degrees.
#define radiansToDegrees(angleRadians) (angleRadians * 180.0 / M_PI)

class RgoHandler3d : public pangolin::Handler3D {
   public:
    enum class KeysEnum : unsigned char {
        AXIS_ALIGN_YX = 'a',
        AXIS_ALIGN_XZ = 'd',
        AXIS_ALIGN_YZ = 'x',
        ROLL_NEG = 'q',
        ROLL_POS = 'e',
        PITCH_NEG = 'w',
        PITCH_POS = 's',
        YAW_NEG = 'z',
        YAW_POS = 'c',
        TOGGLE_WORLD_TO_MODEL = 'm'
    };

    RgoHandler3d::RgoHandler3d(pangolin::OpenGlRenderState& cam_state)
        : pangolin::Handler3D(cam_state) {}
    void Keyboard(pangolin::View& view, unsigned char key, int x, int y,
                  bool pressed) {
        if (!pressed) {
            return;
        }
        auto rotation_requested{false};
        Eigen::Matrix4d transform;
        transform.setIdentity();
        const Eigen::Matrix4d mvmat = cam_state->GetModelViewMatrix();
        Eigen::Quaternion<double> R_env_cam =
            Eigen::Quaternion<double>(mvmat.block<3, 3>(0, 0).transpose());
        R_env_cam.normalize();

        switch (key) {
            case pangolin::PANGO_SPECIAL + pangolin::PANGO_KEY_LEFT:
                transform.block<3, 1>(0, 3) =
                    R_env_cam * Eigen::Vector3d::UnitX() * move_factor_;
                break;
            case pangolin::PANGO_SPECIAL + pangolin::PANGO_KEY_RIGHT:
                transform.block<3, 1>(0, 3) =
                    R_env_cam * -Eigen::Vector3d::UnitX() * move_factor_;
                break;
            case pangolin::PANGO_SPECIAL + pangolin::PANGO_KEY_UP:
                transform.block<3, 1>(0, 3) =
                    R_env_cam * Eigen::Vector3d::UnitZ() * move_factor_;
                break;
            case pangolin::PANGO_SPECIAL + pangolin::PANGO_KEY_DOWN:
                transform.block<3, 1>(0, 3) =
                    R_env_cam * -Eigen::Vector3d::UnitZ() * move_factor_;
                break;
            case KeysEnum::AXIS_ALIGN_YX:
                transform.block<3, 3>(0, 0) = R_env_cam.matrix();
                break;
            case KeysEnum::AXIS_ALIGN_XZ:
                transform.block<3, 3>(0, 0) =
                    R_env_cam.matrix() *
                    Eigen::AngleAxis<double>(degreesToRadians(90),
                                             Eigen::Vector3d::UnitX())
                        .matrix();
                break;
            case KeysEnum::AXIS_ALIGN_YZ:
                transform.block<3, 3>(0, 0) =
                    R_env_cam *
                    Eigen::AngleAxis<double>(degreesToRadians(90),
                                             Eigen::Vector3d::UnitY())
                        .matrix();
                break;
            case KeysEnum::ROLL_NEG:
                transform.block<3, 3>(0, 0) =
                    Eigen::AngleAxis<double>(
                        degreesToRadians(-rotation_factor_deg_),
                        (R_env_cam * Eigen::Vector3d::UnitZ()).normalized())
                        .matrix();
                rotation_requested = true;
                break;
            case KeysEnum::ROLL_POS:
                transform.block<3, 3>(0, 0) =
                    Eigen::AngleAxis<double>(
                        degreesToRadians(rotation_factor_deg_),
                        (R_env_cam * Eigen::Vector3d::UnitZ()).normalized())
                        .matrix();
                rotation_requested = true;
                break;
            case KeysEnum::PITCH_NEG:
                transform.block<3, 3>(0, 0) =
                    Eigen::AngleAxis<double>(
                        degreesToRadians(-rotation_factor_deg_),
                        (R_env_cam * Eigen::Vector3d::UnitX()).normalized())
                        .matrix();
                rotation_requested = true;
                break;
            case KeysEnum::PITCH_POS:
                transform.block<3, 3>(0, 0) =
                    Eigen::AngleAxis<double>(
                        degreesToRadians(rotation_factor_deg_),
                        (R_env_cam * Eigen::Vector3d::UnitX()).normalized())
                        .matrix();
                rotation_requested = true;
                break;
            case KeysEnum::YAW_NEG:
                transform.block<3, 3>(0, 0) =
                    Eigen::AngleAxis<double>(
                        degreesToRadians(-rotation_factor_deg_),
                        (R_env_cam * Eigen::Vector3d::UnitY()).normalized())
                        .matrix();
                rotation_requested = true;
                break;
            case KeysEnum::YAW_POS:
                transform.block<3, 3>(0, 0) =
                    Eigen::AngleAxis<double>(
                        degreesToRadians(rotation_factor_deg_),
                        (R_env_cam * Eigen::Vector3d::UnitY()).normalized())
                        .matrix();
                rotation_requested = true;
                break;
            case KeysEnum::TOGGLE_WORLD_TO_MODEL:
                cam_around_world_ = !cam_around_world_;
                break;
        }

        Eigen::Matrix4d model_view = cam_around_world_ && rotation_requested
                                         ? transform * mvmat
                                         : mvmat * transform;
        cam_state->SetModelViewMatrix(model_view);
    }

   private:
    bool cam_around_world_{false};
    float move_factor_ = 0.4f;
    float rotation_factor_deg_ = 5.f;
};

namespace gl {
static void DrawAxisGizmo(const Eigen::Matrix4d& mvmat,
                          Eigen::Vector3d pos = {0.8, 0.8, 0},
                          float line_width = 2.5f, float axis_len = .075f) {
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    Eigen::Matrix4d transformation;
    transformation.setIdentity();
    transformation.block<3, 3>(0, 0) = mvmat.block<3, 3>(0, 0);
    transformation.block<3, 1>(0, 3) = pos;

    glPushMatrix();
    glMultMatrixd(transformation.data());
    glPushAttrib(GL_LINE_BIT);
    glLineWidth(line_width);
    pangolin::glDrawAxis(axis_len);
    glPopAttrib();
    glPopMatrix();
}

Eigen::Matrix4f lookAt(const Eigen::Vector3f& eye,
                       const Eigen::Vector3f& target,
                       const Eigen::Vector3f& up) {
    Eigen::Vector3f forward = eye - target;
    forward.normalize();
    Eigen::Vector3f left = up.cross(forward);
    left.normalize();

    Eigen::Matrix3f rotation;
    rotation << left, forward.cross(left), forward;

    Eigen::Matrix4f transformation;
    transformation.setIdentity();
    transformation.block<3, 3>(0, 0) = rotation.transpose();
    transformation.block<3, 1>(0, 3) =
        transformation.block<3, 3>(0, 0) * (-eye);

    return transformation;
}
}  // namespace gl

int main(int /*argc*/, char** /*argv*/) {
    const auto width = 640;
    const auto height = 480;

    auto& window = pangolin::CreateWindowAndBind("Main", width, height);

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    const char* glsl_version = "#version 130";
    ImGui_ImplOpenGL3_Init(glsl_version);

    // Setup Dear ImGui style
    ImGui::StyleColorsClassic();
    // Define Projection and initial ModelView matrix
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(width, height, 420, 420, 320, 240, 0.2, 100),
        pangolin::ModelViewLookAt(-12, 2, -12, 0, 0, 0, pangolin::AxisY));

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& d_cam =
        pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0,
                       -640.0f / 480.0f)
            .SetHandler(new RgoHandler3d(s_cam));

    while (!pangolin::ShouldQuit()) {
        // Clear screen and activate view to render into
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        gl::DrawAxisGizmo(s_cam.GetModelViewMatrix());
        glEnable(GL_DEPTH_TEST);

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplWin32_NewFrame();
        {
            ImGui::NewFrame();
            static float f = 0.0f;
            static int counter = 0;

            ImGui::Begin("UI");
            ImGui::SliderFloat(
                "float", &f, 0.0f,
                1.0f);  // Edit 1 float using a slider from 0.0f to 1.0f
            if (ImGui::Button(
                    "Button"))  // Buttons return true when clicked (most
                                // widgets return true when edited/activated)
                counter++;
            ImGui::SameLine();
            ImGui::Text("counter = %d", counter);
            ImGui::Text("Application average %.3f ms/frame (%.1f FPS)",
                        1000.0f / ImGui::GetIO().Framerate,
                        ImGui::GetIO().Framerate);
            ImGui::End();
            ImGui::EndFrame();
        }

        d_cam.Activate(s_cam);

        // Render OpenGL Cube
        pangolin::glDrawColouredCube();
        pangolin::glDrawAxis(1.f);
        pangolin::glDraw_y0(10.0, 10);

        // Swap frames and Process Events
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        pangolin::FinishFrame();
        using namespace std::chrono_literals;
        std::this_thread::sleep_for(16ms);
    }
    ImGui_ImplOpenGL3_Shutdown();

    return 0;
}
