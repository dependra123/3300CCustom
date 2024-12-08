#include "odom.hpp"
#include "main.h"
#include <cmath>

Odometry::Pose currPose(0, 0, 0);
Odometry::Pose prevPose(0, 0, 0);


double prevVert = 0;
double prevHorz = 0;
double prevIMU = 0;

Odometry::OdomTracker::OdomTracker(pros::Rotation* rota, double diameter, double trackingDist){
    this->rota = rota;
    this->radius=diameter/2;
    this->trackingDist=trackingDist;
}
void Odometry::OdomTracker::resetTrackers(){
    this->rota->reset_position();
}
double Odometry::OdomTracker::getTraveled(){
    return rota->get_angle()*this->radius*M_PI/18000;
}
double Odometry::OdomTracker::getTrackDist(){
    return trackingDist;
}

Odometry::Pose::Pose(double x, double y, double theta){
    this->x=x;
    this->y=y;
    this->theta=theta;
}

Odometry::Odom::Odom(OdomTracker* vertTrack, OdomTracker* horzTrack, pros::Imu* imu) {
    this->vertTrack = vertTrack;
    this->horzTrack = horzTrack;
    this->imu = imu;

    // Initialize 1000 particles around the initial pose
    std::normal_distribution<double> distX(currPose.x, 0.1); // Small noise in x
    std::normal_distribution<double> distY(currPose.y, 0.1); // Small noise in y
    std::normal_distribution<double> distTheta(currPose.theta, 1.0); // Small noise in theta

    for (int i = 0; i < 1000; ++i) {
        particles.push_back({distX(generator), distY(generator), distTheta(generator), 1.0});
    }
}



double degToRad(double degrees) {
    return degrees * (M_PI / 180.0);
}

double radToDeg(double radians) {
    return radians * (180.0 / M_PI);
}
void cartesianToPolar(double x, double y, double& r, double& theta) {
    r = sqrt(x * x + y * y);          
    theta = atan2(y, x);              
}
void polarToCartesian(double r, double theta, double& x, double& y) {
    x = r * cos(theta); 
    y = r * sin(theta);  
}
void Odometry::Odom::setPose(Odometry::Pose pos){
    currPose = pos;
}
Odometry::Pose Odometry::Odom::getPose(){
    return currPose;
}
void Odometry::Odom::update(){
    double vert = vertTrack->getTraveled();
    double horz = horzTrack->getTraveled();
    double IMU = degToRad(imu->get_heading());


    double deltaHeading = IMU - prevIMU;

    double avgHeading = degToRad(currPose.theta) + deltaHeading/2;

    double deltaX = horz - prevHorz;
    double deltaY = vert - prevVert;

    double localX = 0;
    double localY = 0;
    if(deltaHeading == 0){
        localX = deltaX;
        localY = deltaY;
    }
    else{
        localX = 2*sin(deltaHeading/2) * (deltaX/deltaHeading + horzTrack->getTrackDist());
        localY = 2*sin(deltaHeading/2) * (deltaX/deltaHeading + vertTrack->getTrackDist());
    }

    double radius = 0;
    double theta = 0;
    double globalx = 0;
    double globaly=0;
    
    cartesianToPolar(localX, localY, radius, theta);
    polarToCartesian(radius, theta-avgHeading, globalx, globaly);
    currPose.x+=globalx;
    currPose.y+=globaly;
    currPose.theta += radToDeg(deltaHeading);

    prevHorz = horz;
    prevVert = vert;
    prevIMU = IMU;
}

//partical filter implmentation
// void Odometry::Odom::update() {
//     double vert = vertTrack->getTraveled();
//     double horz = horzTrack->getTraveled();
//     double IMU = degToRad(imu->get_heading());

//     double deltaHeading = IMU - prevIMU;
//     double avgHeading = degToRad(currPose.theta) + deltaHeading / 2;

//     double deltaX = horz - prevHorz;
//     double deltaY = vert - prevVert;

//     // Predict Step: Update particles based on motion model
//     std::normal_distribution<double> noise(0, 0.01); 
//     for (auto& particle : particles) {
//         double localX = deltaX + noise(generator);
//         double localY = deltaY + noise(generator);

//         particle.x += localY * sin(particle.theta + avgHeading);
//         particle.y += localY * cos(particle.theta + avgHeading);
//         particle.x += localX * -cos(particle.theta + avgHeading);
//         particle.y += localX * sin(particle.theta + avgHeading);
//         particle.theta += deltaHeading + noise(generator);
//     }

//     // Update Weights: Compare particles to IMU 
//     double sensorHeading = IMU; 
//     for (auto& particle : particles) {
//         double headingError = fabs(particle.theta - sensorHeading);
//         particle.weight = exp(-headingError * headingError / 0.05); // Gaussian weight
//     }
    

//     // Extract weights from particles
//     std::vector<double> weights;
//     weights.reserve(particles.size());
//     for (const Particle& p : particles) {
//         weights.push_back(p.weight);
//     }

//     // Create the discrete distribution with weights
//     std::discrete_distribution<> dist(weights.begin(), weights.end());

//     // Resample particles
//     std::vector<Particle> newParticles;
//     newParticles.reserve(particles.size());
//     for (size_t i = 0; i < particles.size(); ++i) {
//         int idx = dist(generator); 
//         newParticles.push_back(particles[idx]);
//     }

//     // Replace old particles with the new set
//     particles = std::move(newParticles);
//     // Estimate Pose: Compute weighted average of particles
//     double sumX = 0, sumY = 0, sumTheta = 0, sumWeights = 0;
//     for (const auto& particle : particles) {
//         sumX += particle.x * particle.weight;
//         sumY += particle.y * particle.weight;
//         sumTheta += particle.theta * particle.weight;
//         sumWeights += particle.weight;
//     }
//     currPose.x = sumX / sumWeights;
//     currPose.y = sumY / sumWeights;
//     currPose.theta = sumTheta / sumWeights;

//     // Update previous values
//     prevHorz = horz;
//     prevVert = vert;
//     prevIMU = IMU;
// }

void Odometry::Odom::init(){
    pros::Task odomTask([=]{
        while(true){
            update();
            pros::delay(10);
        }
    });
}
