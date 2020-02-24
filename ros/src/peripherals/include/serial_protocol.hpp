#ifndef SERIAL_PROTOCOL_H
#define SERIAL_PROTOCOL_H

#include <serial/serial.h>
#include <memory>
#include "protocol.h"
#include "UARTProtocol.h"


class serial_protocol
{
public:
    serial_protocol(std::unique_ptr<serial::Serial> connection);
    ~serial_protocol(void);
    bool send_and_receive(const protocol_MID_E messageID, const protocol_allMessages_U &messageToSend, const uint8_t messageLength, protocol_message_S &receivedMessage);
    void send_no_receive(const protocol_MID_E messageID, const protocol_allMessages_U &messageToSend, const uint8_t messageLength);
    bool request_data_message(const protocol_PBMessageRequest_message_E messageToRequest, protocol_allMessages_U &receivedMessage, protocol_MID_E expectedMID);
private:
    std::unique_ptr<serial::Serial> connection = nullptr;
};



#endif // SERIAL_PROTOCOL_H
