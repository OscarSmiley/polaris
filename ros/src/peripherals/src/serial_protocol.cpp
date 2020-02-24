
#include "serial_protocol.hpp"

serial_protocol::serial_protocol(std::unique_ptr<serial::Serial> connection)
{
    if(connection)
    {
        this->connection = std::move(connection);
    }
}

serial_protocol::~serial_protocol(void)
{
    if(connection)
    {
        connection->close();
    }
}

bool serial_protocol::send_and_receive(const protocol_MID_E messageID, const protocol_allMessages_U &messageToSend, const uint8_t messageLength, protocol_message_S &receivedMessage)
{
    bool ret = false;

    if(connection)
    {
        if((messageLength > 0U) && (messageLength <= PROTOCOL_MAX_MESSAGE_SIZE))
        {
            // Send message
            connection->flushInput();
            send_no_receive(messageID, messageToSend, messageLength);

            UARTProtocol_protocol_S RXFrame;
            connection->read((uint8_t * const)&RXFrame.header, sizeof(RXFrame.header));
            const auto expected_num_bytes = RXFrame.header.length + sizeof(RXFrame.data.crc);
            const auto bytes_read = connection->read((uint8_t * const)&RXFrame.data, expected_num_bytes);

            if(bytes_read == expected_num_bytes)
            {
                std::memcpy(&receivedMessage, RXFrame.data.payload, RXFrame.header.length);
                ret = true;
            }
        }
    }

    return ret;
}

void serial_protocol::send_no_receive(const protocol_MID_E messageID, const protocol_allMessages_U &messageToSend, const uint8_t messageLength)
{
    if(connection)
    {
        // package message in UART frame
        UARTProtocol_protocol_S TXFrame;
        TXFrame.header.length = messageLength + sizeof(messageID);
        TXFrame.data.crc = 0U;
        std::memcpy(&TXFrame.data.payload[0U], &messageID, sizeof(messageID));
        std::memcpy(&TXFrame.data.payload[sizeof(messageID)], &messageToSend, messageLength);
        const uint8_t frame_length = sizeof(TXFrame.header.length) + sizeof(TXFrame.data.crc) + TXFrame.header.length;

        // Send frame out through UART
        connection->flushOutput();

        connection->write((const uint8_t * const)&TXFrame, frame_length);
    }
}

bool serial_protocol::request_data_message(const protocol_PBMessageRequest_message_E messageToRequest, protocol_allMessages_U &receivedMessage, protocol_MID_E expectedMID)
{
    bool ret = false;

    if(connection)
    {
        if(messageToRequest < PROTOCOL_PB_MESSAGE_REQUEST_MESSAGE_COUNT)
        {
            protocol_allMessages_U messageToSend;
            protocol_message_S receiveBuffer;
            messageToSend.POLARIS_messageRequest.requestedMessage = messageToRequest;
            ret = send_and_receive(protocol_MID_POLARIS_PBMessageRequest, messageToSend, sizeof(protocol_PBMessageRequest_S), receiveBuffer);
            
            if(receiveBuffer.messageID == expectedMID)
            {
                receivedMessage = receiveBuffer.message;
                ret = true;
            }
        }
    }

    return ret;
}
