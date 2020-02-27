#include <pangolin/pangolin.h>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <chrono>
class RgoHandler3d : public pangolin::Handler3D {
   public:
    RgoHandler3d::RgoHandler3d(pangolin::OpenGlRenderState& cam_state)
        : pangolin::Handler3D(cam_state) {
        }
    void Keyboard(pangolin::View& view, unsigned char key, int x, int y,
                  bool pressed) {
        if (!pressed) {
            return;
        }

        Eigen::Matrix4d transform;
        transform.setIdentity();
        const Eigen::Matrix4d mvmat = cam_state->GetModelViewMatrix();
        const Eigen::Matrix3d R_env_cam =
            mvmat.block<3, 3>(0, 0).cast<double>().transpose();
        switch (key) {
            case pangolin::PANGO_SPECIAL + pangolin::PANGO_KEY_LEFT:
            transform.block<3, 1>(0, 3) = R_env_cam * unit_vec_left_* move_factor_;
            break;
            case pangolin::PANGO_SPECIAL + pangolin::PANGO_KEY_RIGHT:
            transform.block<3, 1>(0, 3) = -R_env_cam * unit_vec_left_* move_factor_;
            break;
            case pangolin::PANGO_SPECIAL + pangolin::PANGO_KEY_UP:
            transform.block<3, 1>(0, 3) = R_env_cam * unit_vec_forward_* move_factor_;
            break;
            case pangolin::PANGO_SPECIAL + pangolin::PANGO_KEY_DOWN:
            transform.block<3, 1>(0, 3) = -R_env_cam * unit_vec_forward_* move_factor_;
            break;
        }
        cam_state->SetModelViewMatrix(static_cast<Eigen::Matrix4d>(mvmat * transform));
    }

   private:
    Eigen::Vector3d unit_vec_forward_{0, 0, 1};
    Eigen::Vector3d unit_vec_left_{1, 0, 0};
    float move_factor_ = 0.4f;
};

int main(int /*argc*/, char** /*argv*/) {
    pangolin::CreateWindowAndBind("Main", 640, 480);
    glEnable(GL_DEPTH_TEST);

    // Define Projection and initial ModelView matrix
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(640, 480, 420, 420, 320, 240, 0.2, 100),
        pangolin::ModelViewLookAt(-12, 2, -12, 0, 0, 0, pangolin::AxisY));

    bool axis_align = false;
    pangolin::RegisterKeyPressCallback('q',
                                       [&]() { axis_align = !axis_align; });
    // pangolin::RegisterKeyPressCallback('w', [&](){show_x0 = !show_x0;});
    // pangolin::RegisterKeyPressCallback('e', [&](){show_y0 = !show_y0;});
    // pangolin::RegisterKeyPressCallback('r', [&](){show_z0 = !show_z0;});

    // Create Interactive View in window
    pangolin::Handler3D handler(s_cam);
    const int UI_WIDTH = 180;
    pangolin::View& d_cam =
        pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(UI_WIDTH), 1.0,
                       -640.0f / 480.0f)
            .SetHandler(new RgoHandler3d(s_cam));

    // A Panel is just a View with a default layout and input handling
    pangolin::CreatePanel("ui").SetBounds(0.0, 1.0, 0.0,
                                          pangolin::Attach::Pix(UI_WIDTH));
    pangolin::Var<double> center_x("ui.center z", 0, -30, 30);
    pangolin::Var<double> center_y("ui.center y", 0, -30, 30);
    pangolin::Var<double> center_z("ui.center x", 0, -30, 30);
    pangolin::Var<double> view_x("ui.view x", 1, -30, 30);
    pangolin::Var<double> view_y("ui.view y", -2, -30, 30);
    pangolin::Var<double> view_z("ui.view z", 1, -30, 30);

    while (!pangolin::ShouldQuit()) {
        // Clear screen and activate view to render into
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);

        // Render OpenGL Cube
        pangolin::glDrawColouredCube();
        pangolin::glDrawAxis(1.f);
        pangolin::glDraw_y0(10.0, 10);

        // Swap frames and Process Events
        pangolin::FinishFrame();
        using namespace std::chrono_literals;
        std::this_thread::sleep_for(16ms);
    }

    return 0;
}
