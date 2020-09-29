#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <Eigen/Dense>
#include <iostream>
#include <opencv2/opencv.hpp>

const double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();//单位矩阵

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    Eigen::Matrix4f rotation;
    float angle=rotation_angle*MY_PI/180;
    rotation << cos(angle),-sin(angle),0,0,
    sin(angle),cos(angle),0,0,0,0,1,0,0,0,0,1;
    model = rotation * model;

    return model;
}

Eigen::Matrix4f get_rotation(Vector4f axis,float rotation_angle){
    Eigen::Matrix4f R=Eigen::Matrix4f::Identity();
    Eigen::Matrix4f N;
    Eigen::Matrix4f model=Eigen::Matrix4f::Identity();
    float angel=rotation_angle*MY_PI/180;
    N<<0,-axis(2,0),axis(1,0),0,axis(2,0),0,-axis(0,0),0,-axis(1,0),axis(0,0),0,0,0,0,0,1;
    R=cos(angel)*R+(1-cos(angel))*axis*axis.transpose()+sin(angel)*N;
    //error: static_assert failed due to requirement '(int(Eigen::internal::size_of_xpr_at_compile_time<Eigen::Matrix<float, 3, 3, 0, 3, 3>
    model=R*model;
    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function
    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f persp2ortho,ortho_translate,ortho_scale;
    float eye_fov_angle=eye_fov*MY_PI/180;
    float n=zNear;//eye_pos in on 5 in z, so the frustum must be in the -z position
    float f=zFar;
    float b=abs(n)*tan(0.5*eye_fov_angle);
    float t=-b;
    float l=b*aspect_ratio;
    float r=-l;
    persp2ortho<<n,0,0,0,0,n,0,0,0,0,n+f,-n*f,0,0,1,0;
    ortho_scale<<2/(r-l),0,0,0,0,2/(t-b),0,0,0,0,2/(n-f),0,0,0,0,1;
    ortho_translate<<1,0,0,-(r+l)/2,0,1,0,-(t+b)/2,0,0,1,-(n+f)/2,0,0,0,1;

    projection=ortho_scale*ortho_translate*persp2ortho*projection;
    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos;// = {0, 0, 5};
    Eigen::Vector4f axis;//rotation axis
    eye_pos<<0,0,5;
    axis<<0,1,1,0;

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;
    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        //r.set_model(get_model_matrix(angle));
        r.set_model(get_rotation(axis,angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        //r.set_model(get_model_matrix(angle));
        r.set_model(get_rotation(axis,angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
