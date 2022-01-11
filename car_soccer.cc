/** CSci-4611 Assignment 2:  Car Soccer
 */

#include "car_soccer.h"
#include "config.h"


CarSoccer::CarSoccer() : GraphicsApp(1024, 768, "Car Soccer"), use_mouse_(false) {
    // Define a search path for finding data files (images and shaders)
    searchPath_.push_back(".");
    searchPath_.push_back("./data");
    searchPath_.push_back(DATA_DIR_INSTALL);
    searchPath_.push_back(DATA_DIR_BUILD);
}

CarSoccer::~CarSoccer() {
}


Vector2 CarSoccer::joystick_direction() {
    Vector2 dir = Vector2(0,0);
        if (IsKeyDown(GLFW_KEY_LEFT))
            dir[0]--;
        if (IsKeyDown(GLFW_KEY_RIGHT))
            dir[0]++;
        if (IsKeyDown(GLFW_KEY_UP))
            dir[1]++;
        if (IsKeyDown(GLFW_KEY_DOWN))
            dir[1]--;
    return dir;
}


Vector3 CarSoccer::launch_ball(){
    return Vector3(float(std::rand() % 100) - 50 , float(std::rand() % 100) - 50, float(std::rand() % 100) - 50);
}

void CarSoccer::OnSpecialKeyDown(int key, int scancode, int modifiers) {
    if (key == GLFW_KEY_SPACE) {
        // Here's where you could call some form of launch_ball();
        ball_.set_position(Point3(0, 20, 0));
        ball_.set_velocity(CarSoccer::launch_ball());
    }
}


void CarSoccer::UpdateSimulation(double timeStep) {
    // Here's where you shound do your "simulation", updating the positions of the
    // car and ball as needed and checking for collisions.  Filling this routine
    // in is the main part of the assignment.

    //update ball position
    ball_.set_position(ball_.velocity() * timeStep + ball_.position());

    // account for gravity
    Vector3 gravity = Vector3(ball_.velocity().x(), ball_.velocity().y() - (9.8 / 30), ball_.velocity().z());
    ball_.set_velocity(gravity);

    // Check if ball has it either the x, y or z boundries
    // If so bounce off given wall and multiply velocity by 0.8 to simulate friction
    if (ball_.position().x() < (-40 + 2.6))
    {
        ball_.set_position(Point3((-40 + 2.6), ball_.position().y(), ball_.position().z()));
        Vector3 v = Vector3(ball_.velocity().x() * -1, ball_.velocity().y(), ball_.velocity().z());
        ball_.set_velocity(v * 0.8);
    }
    else if (ball_.position().y() < 2.6)
    {
        ball_.set_position(Point3(ball_.position().x(), 2.6, ball_.position().z()));
        Vector3 v = Vector3(ball_.velocity().x(), ball_.velocity().y() * -1, ball_.velocity().z());
        ball_.set_velocity(v * 0.8);
    }
    else if (ball_.position().z() < (-50 + 2.6))
    {
        ball_.set_position(Point3(ball_.position().x(), ball_.position().y(), (-50 + 2.6)));
        Vector3 v = Vector3(ball_.velocity().x(), ball_.velocity().y(), ball_.velocity().z() * -1);
        ball_.set_velocity(v * 0.8);
    }
    else if (ball_.position().x() > (40 - 2.6))
    {
        ball_.set_position(Point3((40 - 2.6), ball_.position().y(), ball_.position().z()));
        Vector3 v = Vector3(ball_.velocity().x() * -1, ball_.velocity().y(), ball_.velocity().z());
        ball_.set_velocity(v * 0.8);
    }
    else if (ball_.position().y() > (35 - 2.6))
    {
        ball_.set_position(Point3(ball_.position().x(), (35 - 2.6), ball_.position().z()));
        Vector3 v = Vector3(ball_.velocity().x(), ball_.velocity().y() * -1, ball_.velocity().z());
        ball_.set_velocity(v * 0.8);
    }
    else if (ball_.position().z() > (50 - 2.6))
    {
        ball_.set_position(Point3(ball_.position().x(), ball_.position().y(), (50 - 2.6)));
        Vector3 v = Vector3(ball_.velocity().x(), ball_.velocity().y(), ball_.velocity().z() * -1);
        ball_.set_velocity(v * 0.8);
    }
    
    // car model handeling
    Vector2 direction = joystick_direction();
    float drag = (0.5 * 0.3 * 1.225 * 6 * car_.speed() * car_.speed());
    if (car_.speed() >= 0)
        car_.set_speed((car_.speed() + direction.y() * 0.1 - drag)  );
    else if (car_.speed() < 0)
        car_.set_speed((car_.speed() + direction.y() * 0.1 + drag) );

    car_.set_angle(direction.x() + car_.angle());

    Matrix4 trans_matrix = Matrix4::Translation(Vector3(-car_.position().x(), 0, -car_.position().z()));
    Matrix4 rot_matrix = Matrix4::RotationY(GfxMath::ToRadians(car_.angle()));
    Matrix4 back_trans_matrix = Matrix4::Translation(Vector3(car_.position().x(), 0, car_.position().z()));

    Point3 car_pos = Point3(car_.position().x(), car_.position().y(), car_.position().z() + car_.speed());

    Matrix4 M = back_trans_matrix * rot_matrix * trans_matrix;

    car_.set_position(M * car_pos);

    // check if the car is out of bounds
    if (car_.position().z() > 48)
    {
        car_.set_position(Point3(car_.position().x(), car_.position().y(), 48));
    }
    else if (car_.position().z() < -48)
    {
        car_.set_position(Point3(car_.position().x(), car_.position().y(), -48));
    }
    if (car_.position().x() > 38)
    {
        car_.set_position(Point3(38, car_.position().y(), car_.position().z()));
    }
    else if (car_.position().x() < -38)
    {
        car_.set_position(Point3(-38, car_.position().y(), car_.position().z()));
    }
    if (car_.position().y() != 1)
    {
        car_.set_position(Point3(car_.position().x(), 1, car_.position().z()));
    }

    // car and ball collision
    Vector3 distance_vec_apart = ball_.position() - car_.position();
    float noIntersectDist = car_.collision_radius() + ball_.radius();

    if (distance_vec_apart.Length() <= noIntersectDist)
    {
        ball_.set_position(Point3(ball_.position().x() + car_.speed() * sin(car_.angle()), ball_.position().y(), ball_.position().z() + car_.speed() * cos(car_.angle())));
        ball_.set_velocity(Vector3(ball_.velocity().x() + car_.speed() * sin(car_.angle()), ball_.velocity().y(), ball_.velocity().z() + car_.speed() * cos(car_.angle())));
    }

    //reset if ball is scored
    bool is_in_goal1 = ball_.position().z() < (-50 + 3) && (ball_.position().x() < 10 && ball_.position().x() > -10) && ball_.position().y() < 20;
    bool is_in_goal2 = ball_.position().z() > (50 - 3) && (ball_.position().x() < 10 && ball_.position().x() > -10) && ball_.position().y() < 20;

    if (is_in_goal1 || is_in_goal2)
    {
        ball_.Reset();
        car_.Reset();
        launch_ball();
    }
}


void CarSoccer::InitOpenGL() {
    // Set up the camera in a good position to see the entire field
    projMatrix_ = Matrix4::Perspective(60, aspect_ratio(), 1, 1000);
    modelMatrix_ = Matrix4::LookAt(Point3(0,60,70), Point3(0,0,10), Vector3(0,1,0));
 
    // Set a background color for the screen
    glClearColor(0.8f, 0.8f, 0.8f, 1.0f);
    
    // Load some image files we'll use
    fieldTex_.InitFromFile(Platform::FindFile("pitch.png", searchPath_));
    crowdTex_.InitFromFile(Platform::FindFile("crowd.png", searchPath_));
}


void CarSoccer::DrawUsingOpenGL() {
    // Draw the crowd as a fullscreen background image
    quickShapes_.DrawFullscreenTexture(Color(1,1,1), crowdTex_);
    
    // Draw the field with the field texture on it.
    Color col(16.0f/255.0f, 46.0f/255.0f, 9.0f/255.0f);
    Matrix4 M = Matrix4::Translation(Vector3(0.0f, -0.201f, 0.0f)) * Matrix4::Scale(Vector3(50.0f, 1.0f, 60.0f));
    quickShapes_.DrawSquare(modelMatrix_ * M, viewMatrix_, projMatrix_, col);
    M = Matrix4::Translation(Vector3(0.0f, -0.2f, 0.0f)) * Matrix4::Scale(Vector3(40.0f, 1.0f, 50.0f));
    quickShapes_.DrawSquare(modelMatrix_ * M, viewMatrix_, projMatrix_, Color(1,1,1), fieldTex_);
    
    // Draw the car
    Color carcol(0.8f, 0.2f, 0.2f);
    Matrix4 Mcar =
        Matrix4::Translation(car_.position() - Point3(0,0,0)) *
        Matrix4::Scale(car_.size()) *
        Matrix4::Scale(Vector3(0.5f, 0.5f, 0.5f));
    quickShapes_.DrawCube(modelMatrix_ * Mcar, viewMatrix_, projMatrix_, carcol);
    
    
    // Draw the ball
    Color ballcol(1,1,1);
    Matrix4 Mball =
        Matrix4::Translation(ball_.position() - Point3(0,0,0)) *
        Matrix4::Scale(Vector3(ball_.radius(), ball_.radius(), ball_.radius()));
    quickShapes_.DrawSphere(modelMatrix_ * Mball, viewMatrix_, projMatrix_, ballcol);
    
    
    // Draw the ball's shadow -- this is a bit of a hack, scaling Y by zero
    // flattens the sphere into a pancake, which we then draw just a bit
    // above the ground plane.
    Color shadowcol(0.2f, 0.4f, 0.15f);
    Matrix4 Mshadow =
        Matrix4::Translation(Vector3(ball_.position()[0], -0.1f, ball_.position()[2])) *
        Matrix4::Scale(Vector3(ball_.radius(), 0, ball_.radius())) *
        Matrix4::RotationX(90);
    quickShapes_.DrawSphere(modelMatrix_ * Mshadow, viewMatrix_, projMatrix_, shadowcol);
    
    
    // You should add drawing the goals and the boundary of the playing area
    // using quickShapes_.DrawLines()
    Color white(1, 1, 1);
    std::vector<Point3> own_goal;
    own_goal.push_back(Point3(10.0, 10.0, 50.0));
    own_goal.push_back(Point3(-10.0, 10.0, 50.0));
    own_goal.push_back(Point3(-10.0, 0.0, 50.0));
    own_goal.push_back(Point3(10.0, 0.0, 50.0));
    quickShapes_.DrawLines(modelMatrix_, viewMatrix_, projMatrix_, white, own_goal, QuickShapes::LinesType::LINE_LOOP, 0.1);

    std::vector<Point3> opp_goal;
    opp_goal.push_back(Point3(10.0, 10.0, -50.0));
    opp_goal.push_back(Point3(-10.0, 10.0, -50.0));
    opp_goal.push_back(Point3(-10.0, 0.0, -50.0));
    opp_goal.push_back(Point3(10.0, 0.0, -50.0));
    quickShapes_.DrawLines(modelMatrix_, viewMatrix_, projMatrix_, white, opp_goal, QuickShapes::LinesType::LINE_LOOP, 0.1);

    std::vector<Point3> right_boundry;
    right_boundry.push_back(Point3(40.0, 0.0, 50.0));
    right_boundry.push_back(Point3(40.0, 35.0, 50.0));
    right_boundry.push_back(Point3(40.0, 35.0, -50.0));
    right_boundry.push_back(Point3(40.0, 0.0, -50.0));
    quickShapes_.DrawLines(modelMatrix_, viewMatrix_, projMatrix_, white, right_boundry, QuickShapes::LinesType::LINE_LOOP, 0.1);
    
    std::vector<Point3> left_boundry;
    left_boundry.push_back(Point3(-40.0, 0.0, 50.0));
    left_boundry.push_back(Point3(-40.0, 35.0, 50.0));
    left_boundry.push_back(Point3(-40.0, 35.0, -50.0));
    left_boundry.push_back(Point3(-40.0, 0.0, -50.0));
    quickShapes_.DrawLines(modelMatrix_, viewMatrix_, projMatrix_, white, left_boundry, QuickShapes::LinesType::LINE_LOOP, 0.1);
    
    std::vector<Point3> back_boundry;
    back_boundry.push_back(Point3(40.0, 0.0, -50.0));
    back_boundry.push_back(Point3(40.0, 35.0, -50.0));
    back_boundry.push_back(Point3(-40.0, 35.0, -50.0));
    back_boundry.push_back(Point3(-40.0, 0.0, -50.0));
    quickShapes_.DrawLines(modelMatrix_, viewMatrix_, projMatrix_, white, back_boundry, QuickShapes::LinesType::LINE_LOOP, 0.1);
    
    std::vector<Point3> front_boundry;
    front_boundry.push_back(Point3(40.0, 0.0, 50.0));
    front_boundry.push_back(Point3(40.0, 35.0, 50.0));
    front_boundry.push_back(Point3(-40.0, 35.0, 50.0));
    front_boundry.push_back(Point3(-40.0, 0.0, 50.0));
    quickShapes_.DrawLines(modelMatrix_, viewMatrix_, projMatrix_, white, front_boundry, QuickShapes::LinesType::LINE_LOOP, 0.1);
}
