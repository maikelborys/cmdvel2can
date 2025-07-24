 #ifndef CMDVEL2CAN_CAN_INTERFACE_HPP
 #define CMDVEL2CAN_CAN_INTERFACE_HPP

#include <string>
#include <mutex>
#include <linux/can.h>

namespace cmdvel2can {

/**
 * @brief Interface for CAN communication using SocketCAN
 * 
 * This class provides a thread-safe interface for sending and receiving
 * CAN frames through the Linux SocketCAN interface.
 */
class CANInterface {
public:
    /**
     * @brief Constructor
     * @param interface_name Name of the CAN interface (e.g., "can0")
     */
    explicit CANInterface(const std::string& interface_name);
    
    /**
     * @brief Destructor - ensures proper cleanup
     */
    ~CANInterface();
    
    /**
     * @brief Initialize the CAN interface
     * @return true if successful, false otherwise
     */
    bool initialize();
    
    /**
     * @brief Send a CAN frame
     * @param frame The CAN frame to send
     * @return true if successful, false otherwise
     */
    bool sendFrame(const can_frame& frame);
    
    /**
     * @brief Receive a CAN frame (non-blocking)
     * @param frame Reference to store received frame
     * @return true if frame received, false if no frame available
     */
    bool receiveFrame(can_frame& frame);
    
    /**
     * @brief Check if interface is connected and operational
     * @return true if operational, false otherwise
     */
    bool isConnected() const;
    
    /**
     * @brief Shutdown the CAN interface
     */
    void shutdown();
    
private:
    std::string interface_name_;
    int socket_fd_;
    bool is_initialized_;
    mutable std::mutex interface_mutex_;
    
    // Disable copy constructor and assignment operator
    CANInterface(const CANInterface&) = delete;
    CANInterface& operator=(const CANInterface&) = delete;
};

} // namespace cmdvel2can

#endif // CMDVEL2CAN_CAN_INTERFACE_HPP
