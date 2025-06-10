#include <dds/dds.h>
#include <cstddef>
#include <cstring>
#include <iostream>
#include <chrono>
#include <thread>
#include <vector>
#include "Message.h"

static uint64_t current_time_ns() {
    using namespace std::chrono;
    return duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count();
}

void fill_payload(std::vector<uint8_t>& payload, uint64_t timestamp_ns) {
    if (payload.size() < sizeof(uint64_t)) return;
    std::memcpy(payload.data(), &timestamp_ns, sizeof(uint64_t));
    for (size_t i = sizeof(uint64_t); i < payload.size(); ++i) {
        payload[i] = static_cast<uint8_t>(i & 0xFF);
    }
}

void usage(const char* prog) {
    std::cerr << "Usage: " << prog << " --payload <bytes> --frequency <Hz> --sample <count>\n";
    std::exit(1);
}

int main(int argc, char** argv) {
    size_t payload_size = 64;
    int sample_count = 1000;
    double frequency = 1000.0;

    // Parse CLI arguments
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--payload") == 0 && i + 1 < argc) {
            payload_size = std::atoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--frequency") == 0 && i + 1 < argc) {
            frequency = std::atof(argv[++i]);
        } else if (std::strcmp(argv[i], "--sample") == 0 && i + 1 < argc) {
            sample_count = std::atoi(argv[++i]);
        } else {
            usage(argv[0]);
        }
    }

    if (frequency < 0.0) {
        std::cerr << "Frequency must be non-negative\n";
        return 1;
    }
    uint64_t sleep_ns = 0;
    if (frequency > 0) {
        sleep_ns = static_cast<uint64_t>(1e9 / frequency);
    }

    dds_entity_t participant = dds_create_participant(DDS_DOMAIN_DEFAULT, nullptr, nullptr);
    if (participant < 0) {
        std::cerr << "dds_create_participant: " << dds_strretcode(-participant) << "\n";
        return 1;
    }

    dds_entity_t topic = dds_create_topic(participant, &MessageModule_DataType_desc, "latency_topic", nullptr, nullptr);
    if (topic < 0) {
        std::cerr << "dds_create_topic: " << dds_strretcode(-topic) << "\n";
        dds_delete(participant);
        return 1;
    }

    dds_qos_t *qos = dds_create_qos ();
    dds_qset_reliability (qos, DDS_RELIABILITY_RELIABLE, DDS_SECS (10));

    dds_entity_t writer = dds_create_writer(participant, topic, qos, nullptr);
    if (writer < 0) {
        std::cerr << "dds_create_writer: " << dds_strretcode(-writer) << "\n";
        dds_delete(participant);
        return 1;
    }

    // Wait for subscriber
    std::cout << "Waiting for subscriber...\n";
    dds_return_t rc = dds_set_status_mask(writer, DDS_PUBLICATION_MATCHED_STATUS);
    if (rc != DDS_RETCODE_OK) {
        std::cerr << "dds_set_status_mask: " << dds_strretcode(-rc) << "\n";
        dds_delete(participant);
        return 1;
    }

    uint32_t status = 0;
    while (!(status & DDS_PUBLICATION_MATCHED_STATUS)) {
        rc = dds_get_status_changes(writer, &status);
        if (rc != DDS_RETCODE_OK) {
            std::cerr << "dds_get_status_changes: " << dds_strretcode(-rc) << "\n";
            dds_delete(participant);
            return 1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    std::cout << "Sending " << sample_count << " samples, payload = " << payload_size << " bytes, rate = " << frequency << " Hz\n";

    int sent = 0;
    MessageModule_DataType msg{};
    msg.payload._length = payload_size;
    msg.payload._release = true;
    std::vector<uint8_t> buffer(payload_size);
    while (sample_count == 0 || sent < sample_count) {
        fill_payload(buffer, current_time_ns());

        msg.payload._buffer = buffer.data();

        rc = dds_write(writer, &msg);
        if (rc < 0) {
            std::cerr << "dds_write: " << dds_strretcode(-rc) << "\n";
            dds_delete(participant);
            return 1;
        }

        if (sleep_ns > 0) {
            std::this_thread::sleep_for(std::chrono::nanoseconds(sleep_ns));
        }
        sent += 1;
    }

    dds_delete(participant);
    return 0;
}
