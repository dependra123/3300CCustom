#include "iostream"
#include "api.h"
#include <vector>
#include <random>

struct Particle {
    double x, y, theta; // State of the particle
    double weight;      // Importance weight
};

struct Odometry {
    class OdomTracker{
        private:
            pros::Rotation* rota = nullptr;
            double radius;
            double trackingDist;

        public:
            OdomTracker(pros::Rotation* rota, double diameter, double trackingDist);
            void resetTrackers();
            double getTraveled();
            double getTrackDist();


    };
    class Pose{            
        public:
            Pose(double x, double y, double theta);
            double x;
            double y;
            double theta;
    };

    class Odom {
        private:
            OdomTracker* vertTrack;
            OdomTracker* horzTrack;
            pros::Imu* imu;
            

        public:
            std::vector<Particle> particles;
            std::default_random_engine generator;

            Odom(OdomTracker* vertTrack, OdomTracker* horzTrack, pros::Imu* imu);
            Pose getPose();
            void setPose(Pose pose);
            void update();
            void init();
    };
};
