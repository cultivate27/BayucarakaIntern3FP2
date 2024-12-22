#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include <thread>
#include <atomic>
#include <mutex>

const float SCALE_CM_PER_STEP_X = 0.02;
const float SCALE_CM_PER_STEP_Y = 0.038;
const float SCALE_CM_PER_PIXEL = 0.15;

float pixelStartX;
float pixelStartY;

std::atomic<bool> processCommand(false);
std::atomic<bool> waitingForData(false);
std::atomic<bool> setPosition(false);
std::mutex dataMutex;
std::vector<float> lastCoordinates(3); 

void detectCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    if (!waitingForData) {
        return;
    }

    if (msg->data.size() != 3) {
        ROS_WARN("Data yang diterima tidak valid. Harus ada 3 elemen (X, Y, Z).");
        return;
    }

    std::lock_guard<std::mutex> lock(dataMutex);
    lastCoordinates[0] = msg->data[0];
    lastCoordinates[1] = msg->data[1];
    lastCoordinates[2] = msg->data[2];

    waitingForData = false;
    ROS_INFO("\n\nData deteksi diterima: (X: %.2f, Y: %.2f, Z: %.2f)", lastCoordinates[0], lastCoordinates[1], lastCoordinates[2]);
}

//101,101

void keyboardInput() {
    char input;
    while (ros::ok()) {
        std::cin >> input;
        if (input == 'o') {
            setPosition = true;
            ROS_INFO("Input 'o' diterima. Mode mengatur titik awal sedang berjalan");
        }
        if (input == 'x') {
            processCommand = true; 
            waitingForData = true;
            ROS_INFO("Input 'x' diterima. Menunggu data deteksi...");
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "control_node");
    ros::NodeHandle nh;

    ros::Subscriber detect_sub = nh.subscribe("object_coordinates", 10, detectCallback);

    ros::Publisher arduino_pub = nh.advertise<std_msgs::String>("stepper_commands", 10);

    std::thread keyboardThread(keyboardInput);

    ROS_INFO("Tekan 'x' untuk inisiasi. Tekan 'o' untuk mengatur titik awal");

    ros::Rate rate(10);
    while (ros::ok()) {
        ros::spinOnce();

        if (processCommand && !waitingForData) {
            processCommand = false;

            float pixelTargetX, pixelTargetY, pixelTargetZ;
            {
                std::lock_guard<std::mutex> lock(dataMutex);

                pixelTargetX = lastCoordinates[0];
                pixelTargetY = lastCoordinates[1];
                pixelTargetZ = lastCoordinates[2];
                
            }
            if (setPosition) {
                pixelStartX = lastCoordinates[0];
                pixelStartY = lastCoordinates[1];
                setPosition = false;
                ROS_INFO("\nInput 'o' diterima. Titik 0 diatur pada X: %.2f, Y: %.2f", pixelStartX, pixelStartY);
            }
            
            /*
            To Be Added:
            - FoV
                1. Ukur jarak vertical antara kamera dan permukaan payload (83 cm)
                2. Cari 1280 pixel dalam dunia nyata (109 cm)

                Rumus:
                    Horizontal FoV = 2 * arctan (1280 pixel dalam dunia nyata / 2 * tinggi kamera ke permukaan payload).
                Jika sulit dihitung dengan rumus, biasanya kamera 720p memiliki FoV: 60 sampai 90

            - Tambahkan konversi pixel ke cm
                Untuk mendapatkan posisi nyata dari payload, dibuthkan data sebagai berikut:
                    1. Field Width = 2 * h * tan (FoV Horizontal / 2)
                    2. Scale cm per pixel = Field Width / Resolution Width (1280)
                
                Setelah mendapatkan scale cm per pixel langkah selanjutnya adalah mengalikannya dengan jarak

            To Be Fixed:
            Step X = (Posisi X payload dalam pixel - Posisi  X awal claw dalam pixel) * scale_cm_per_pixel / 0.02
            Step Y = (Posisi X payload dalam pixel - Posisi awal claw dalam pixel) * scale_cm_per_pixel / 0.038

            cm per pixel sementara = 0.0852
            */
            int stepsX;
            int stepsY;

            if (pixelTargetX > pixelStartX) {
                stepsX = (pixelTargetX * SCALE_CM_PER_PIXEL - pixelStartX * SCALE_CM_PER_PIXEL) / SCALE_CM_PER_STEP_X;
            }
            else if (pixelTargetX < pixelStartX) {
                stepsX = (pixelStartX * SCALE_CM_PER_PIXEL - pixelTargetX * SCALE_CM_PER_PIXEL) / SCALE_CM_PER_STEP_X;
            }
            else {
                stepsX = 0;
            }

            if (pixelTargetY > pixelStartY) {
                stepsY = (pixelTargetY * SCALE_CM_PER_PIXEL - pixelStartY * SCALE_CM_PER_PIXEL) / SCALE_CM_PER_STEP_Y;
            }
            else if (pixelTargetX < pixelStartX) {
                stepsY = (pixelStartY * SCALE_CM_PER_PIXEL - pixelTargetY * SCALE_CM_PER_PIXEL) / SCALE_CM_PER_STEP_Y;
            }
            else {
                stepsY = 0;
            }
            
            ROS_INFO("\nTarget koordinat (X: %.2f, Y: %.2f, Z: %.2f)", pixelTargetX, pixelTargetY, pixelTargetZ);
            ROS_INFO("\nLangkah stepper (X: %d, Y: %d)", stepsX, stepsY);

            std::ostringstream command;
            command << stepsX << "," << stepsY;

            std_msgs::String cmd_msg;
            cmd_msg.data = command.str();
            arduino_pub.publish(cmd_msg);


            ROS_INFO("\nPerintah dikirim ke Arduino: %s", cmd_msg.data.c_str());

            // pixelStartX = pixelTargetX;
            // pixelStartY = pixelTargetY;

            ROS_INFO("\n\nPerintah dikirim. Tekan 'x' untuk mengirim lagi.");
        }

        rate.sleep();
    }

    keyboardThread.join();
    return 0;
}
