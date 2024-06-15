#include <iostream>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>

namespace scan{
    class CANInterface { 
        private:
            int soc;
            struct sockaddr_can addr;
            struct ifreq ifr;
            bool initialized;

        public:
            // Default constructor
            CANInterface() : soc(-1), initialized(false) {}
            ~CANInterface() { if (initialized) {close(soc); } }

            CANInterface(const char* interfaceName) : soc(-1), initialized(false) {
                initialize(interfaceName);
            }

            bool isInitialized() const { return initialized; }

            // Method to initialize the interface
            bool initialize(const char* interfaceName) {
                if (initialized)
                    return true;  // Already initialized
                soc = socket(PF_CAN, SOCK_RAW, CAN_RAW);
                if (soc < 0) {
                    perror("Socket creation failed");
                    return false;
                }
                std::strcpy(ifr.ifr_name, interfaceName);
                ioctl(soc, SIOCGIFINDEX, &ifr);
                std::memset(&addr, 0, sizeof(addr));
                addr.can_family = AF_CAN;
                addr.can_ifindex = ifr.ifr_ifindex;
                if (bind(soc, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
                    perror("Socket bind failed");
                    close(soc);
                    return false;
                }
                initialized = true;
                return true;
            }

            // Method to send a CAN frame
            bool send_frame(uint32_t id, unsigned char *data, unsigned int dlc) {
                if (!initialized) {
                    std::cerr << "Interface not initialized" << std::endl;
                    return false;
                }
                if (dlc > CAN_MAX_DLEN) {
                    std::cerr << "Data length code too large" << std::endl;
                    return false;
                }
                struct can_frame frame;
                frame.can_id = id | CAN_EFF_FLAG; // Set the Extended Frame Format flag
                frame.can_dlc = dlc;
                std::memcpy(frame.data, data, dlc);
                int bytes_sent = write(soc, &frame, sizeof(frame));
                if (bytes_sent != sizeof(frame)) {
                    perror("Write to socket failed");
                    return false;
                }
                return true;
            }
            
            // Method to receive a CAN frame
            bool receive_frame(uint32_t& id, unsigned char* data, unsigned int& dlc) {
                if (!initialized) {
                    std::cerr << "Interface not initialized" << std::endl;
                    return false;
                }
                struct can_frame frame;
                int bytes_received = read(soc, &frame, sizeof(frame));
                if (bytes_received < 0) {
                    perror("Read from socket failed");
                    return false;
                }
                if (bytes_received < sizeof(frame)) {
                    std::cerr << "Incomplete frame received" << std::endl;
                    return false;
                }
                id = static_cast<uint32_t>(frame.can_id & CAN_EFF_MASK);
                dlc = frame.can_dlc;
                std::memcpy(data, frame.data, dlc);
                return true;
            }
    };
}