#include <pangolin/pangolin.h>
#include <Eigen/SVD>
#include <Eigen/Geometry>
#include <chrono>
// class Camera
// {
// public:
//     void move(glm::vec3 directions, glm::vec2 rotations, float frametime);
//     // ... constructors etc.
// private:
//     glm::mat4 view_;
//     glm::vec3 camera_pos_;
//     glm::quat camera_orientation_;
//     const float camera_speed_;
// }

// void Camera::move(glm::vec3 directions, glm::vec2 rotations, float frametime)
// {
//     auto pitch = glm::quat(glm::vec3(-rotations.y, 0, 0.f));
//     auto yaw = glm::quat(glm::vec3(0, -rotations.x, 0.f));

//     camera_orientation_ = glm::normalize(yaw * camera_orientation_ * pitch);

//     auto camera_roll_direction = camera_orientation_ * glm::vec3(0, 0, -1);
//     auto camera_pitch_direction = camera_orientation_ * glm::vec3(-1, 0, 0);

//     // forward/backward move - all axes could be affected
//     camera_pos_ += directions[0] * camera_roll_direction * frametime * camera_speed_;
//     // left and right strafe - only xz could be affected
//     camera_pos_ += directions[1] * camera_pitch_direction * frametime * camera_speed_;
//     // up and down flying - only y-axis could be affected
//     camera_pos_.y += directions[2] * frametime * camera_speed_;

//     view_ = glm::lookAt(camera_pos_, camera_pos_ + camera_roll_direction,
//                         glm::cross(camera_roll_direction, camera_pitch_direction));
// }


int main( int /*argc*/, char** /*argv*/ )
{
    pangolin::CreateWindowAndBind("Main",640,480);
    glEnable(GL_DEPTH_TEST);

    // Define Projection and initial ModelView matrix
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(640,480,420,420,320,240,0.2,100),
        pangolin::ModelViewLookAt(-12,2,-12, 0,0,0, pangolin::AxisY)
    );
    pangolin::AxisDirection  axis_direction;
    bool axis_align = false;
    pangolin::RegisterKeyPressCallback('q', [&](){axis_align = !axis_align;});
    // pangolin::RegisterKeyPressCallback('w', [&](){show_x0 = !show_x0;});
    // pangolin::RegisterKeyPressCallback('e', [&](){show_y0 = !show_y0;});
    // pangolin::RegisterKeyPressCallback('r', [&](){show_z0 = !show_z0;});

    // Create Interactive View in window
    pangolin::Handler3D handler(s_cam);
    const int UI_WIDTH = 180;
    pangolin::View& d_cam = pangolin::CreateDisplay()
        .SetBounds(0.0, 1.0, pangolin::Attach::Pix(UI_WIDTH), 1.0, -640.0f/480.0f)
        .SetHandler(new pangolin::Handler3D(s_cam));

  // A Panel is just a View with a default layout and input handling
    pangolin::CreatePanel("ui")
      .SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(UI_WIDTH));
    pangolin::Var<double> center_x("ui.center z",0,-30,30);
    pangolin::Var<double> center_y("ui.center y",0,-30,30);
    pangolin::Var<double> center_z("ui.center x",0,-30,30);
    pangolin::Var<double> view_x("ui.view x",1,-30,30);
    pangolin::Var<double> view_y("ui.view y",-2,-30,30);
    pangolin::Var<double> view_z("ui.view z",1,-30,30);
    Eigen::Vector3d translate {0., 0., 0.};

    bool move_pressed = false;
    pangolin::RegisterKeyPressCallback(pangolin::PANGO_SPECIAL + pangolin::PANGO_KEY_RIGHT, [&]() {
        std::cout << "RIGHT " << std::endl;
        move_pressed  = true;
        translate[0] = -0.02;
    });
    pangolin::RegisterKeyPressCallback(pangolin::PANGO_SPECIAL + pangolin::PANGO_KEY_LEFT, [&](){
        std::cout << "LEFT " << std::endl;
        move_pressed  = true;
        translate[0] = 0.02;
    });
    pangolin::RegisterKeyPressCallback(pangolin::PANGO_SPECIAL + pangolin::PANGO_KEY_UP, [&](){
        std::cout << "UP " << std::endl;
        move_pressed  = true;
        translate[2] = 0.02;
    });
    pangolin::RegisterKeyPressCallback(pangolin::PANGO_SPECIAL + pangolin::PANGO_KEY_DOWN, [&](){
        std::cout << "DOWN " << std::endl;
        move_pressed  = true;
        translate[2] = -0.02;
    });

    while( !pangolin::ShouldQuit() )
    {
        // Clear screen and activate view to render into
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        // auto mvm = s_cam.GetModelViewMatrix().Translate
        //            (translate[0], translate[1], translate[2]);

        // s_cam.SetModelViewMatrix(mvm);

        // s_cam.Apply();
        const Eigen::Matrix4d mvmat = s_cam.GetModelViewMatrix();
        const Eigen::Matrix3d R_env_cam = mvmat.block<3,3>(0,0).cast<double>().transpose();
        Eigen::Vector3d unit_vec_Z{0, 0, 1};
        auto direction_Z = R_env_cam * unit_vec_Z;

        Eigen::Vector3d unit_vec_X{1, 0, 0};
        auto direction_X = R_env_cam * unit_vec_X;
        if(move_pressed){

            Eigen::Matrix4d transform;
            transform.setIdentity();
            if(translate[2]) {
                transform.block<3, 1>(0, 3) = direction_Z*translate[2];
            }
            if(translate[0]) {
                transform.block<3, 1>(0, 3) = direction_X*translate[0];
            }
            const pangolin::OpenGlMatrix T_vw = s_cam.GetModelViewMatrix() * transform;
            s_cam.SetModelViewMatrix(T_vw);
            move_pressed = false;
        }
        // Eigen::Matrix4d mat =  s_cam.GetModelViewMatrix();

        // Eigen::Affine3d affine = mat.block<3,3>(0,0);
        // Eigen::Transform<double, 4, 4> t(mat);
        // auto rotation = t.affine().rotation();

        // auto ff = rotation.inverse() * Eigen::Vector3d{0,0,1};

         // Add named Panel and bind to variables beginning 'ui'
        translate = Eigen::Vector3d::Zero();
        d_cam.Activate(s_cam);

        // if (axis_align) {

        //     const Eigen::Vector3f center {0.f, 0.f, 0.f};
        //     const Eigen::Vector3f view  {1.f, 0.f, 1.f};
        //     const auto mvm = pangolin::ModelViewLookAt(view[0], view[1], view[2], center[0], center[1], center[2], pangolin::AxisY);
        //     s_cam.SetModelViewMatrix(mvm);
        //     s_cam.Apply();
        //     axis_align = false;
        // }

        // Render OpenGL Cube
        pangolin::glDrawColouredCube();
        pangolin::glDrawAxis(1.f);
        pangolin::glDraw_y0(10.0, 10);
        // Swap frames and Process Events
        pangolin::FinishFrame();
        using namespace std::chrono_literals;
        std::this_thread::sleep_for(16ms);
        //
    }

    return 0;
}
