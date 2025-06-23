#include <dds/dds.h>
#include <dds/ddsc/dds_public_qos.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cassert>
#include <chrono>
#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fstream>
#include <iostream>
#include <ostream>
#include <string>
#include <thread>
#include <vector>

#include "Message.h"

using namespace std;
using Clock = chrono::steady_clock;
using namespace chrono;

static uint64_t current_time_ns() { return duration_cast<nanoseconds>(Clock::now().time_since_epoch()).count(); }

void usage(const char *prog) {
    cerr << "Usage: " << prog
         << " [--payload <bytes>] [--frequency <Hz>] [--sample <count>] [--log <file.csv>] [--warmup <seconds>] "
            "[--listener]"
         << endl;
    exit(1);
}

int main(int argc, char **argv) {
    size_t payload_size = 64;
    int sample_count = 1000;
    double frequency = 1000.0;
    string log_path;
    int warmup_sec = 1;
    bool use_listener = false;

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--payload") && i + 1 < argc) {
            payload_size = atoi(argv[++i]);
        } else if (!strcmp(argv[i], "--frequency") && i + 1 < argc) {
            frequency = atof(argv[++i]);
        } else if (!strcmp(argv[i], "--sample") && i + 1 < argc) {
            sample_count = atoi(argv[++i]);
        } else if (!strcmp(argv[i], "--log") && i + 1 < argc) {
            log_path = argv[++i];
        } else if (!strcmp(argv[i], "--warmup") && i + 1 < argc) {
            warmup_sec = atoi(argv[++i]);
        } else if (!strcmp(argv[i], "--listener")) {
            use_listener = true;
        } else {
            usage(argv[0]);
        }
    }

    cout << "Freq: " << frequency << " Hz, Payload: " << payload_size << " bytes, Samples: " << sample_count << endl;
    cout << "Using waitset or listener: " << (use_listener ? "listener" : "waitset") << endl;

    ofstream logf;
    if (!log_path.empty()) {
        logf.open(log_path);
        if (!logf.is_open()) {
            perror("fopen");
            return 1;
        }
        logf << "Frequency,Payload,RTT" << endl;
    }

    vector<uint64_t> rtts;
    rtts.reserve(sample_count);
    atomic<int> received_samples = 0;

    dds_entity_t participant = dds_create_participant(DDS_DOMAIN_DEFAULT, NULL, NULL);
    dds_entity_t topic = dds_create_topic(participant, &MessageModule_DataType_desc, "latency_topic", NULL, NULL);

    dds_qos_t *qos = dds_create_qos();
    dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_SECS(10));
    dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 10);

    MessageModule_DataType msg{};
    void *samples[1];
    dds_sample_info_t infos[1];
    samples[0] = &msg;

    dds_entity_t reader;
    static bool warmup_finished = false;

    if (use_listener) {
        auto *userdata = new pair<vector<uint64_t> *, atomic<int> *>(&rtts, &received_samples);
        dds_listener_t *listener = dds_create_listener(userdata);

        dds_lset_data_available(listener, [](dds_entity_t rd, void *arg) {
            auto *userdata = static_cast<pair<vector<uint64_t> *, atomic<int> *> *>(arg);
            MessageModule_DataType msg{};
            void *samples[1] = {&msg};
            dds_sample_info_t infos[1];
            auto rc = dds_take(rd, samples, infos, 1, 1);
            if (rc > 0 && infos[0].valid_data && warmup_finished) {
                uint64_t recv_time = current_time_ns();
                uint64_t send_time;
                memcpy(&send_time, msg.payload._buffer, sizeof(uint64_t));
                userdata->first->push_back(recv_time - send_time);
                (*userdata->second)++;
            }
        });

        reader = dds_create_reader(participant, topic, qos, listener);
    } else {
        reader = dds_create_reader(participant, topic, qos, NULL);
    }

    dds_delete_qos(qos);

    cout << "Warming up for " << warmup_sec << " seconds..." << endl << flush;
    if (use_listener) {
        this_thread::sleep_for(seconds(warmup_sec));
        warmup_finished = true;
    } else {
        auto warmup_start = Clock::now();
        auto warmup_dur = seconds(warmup_sec);
        dds_entity_t ws = dds_create_waitset(participant);
        dds_waitset_attach(ws, dds_create_readcondition(reader, DDS_ANY_STATE), reader);
        while (Clock::now() - warmup_start < warmup_dur) {
            dds_attach_t triggered[1];
            dds_waitset_wait(ws, triggered, 1, DDS_SECS(10));
            dds_take(reader, samples, infos, 1, 1);
        }
    }

    cout << "Starting latency measurement..." << endl << flush;

    if (use_listener) {
        while (received_samples.load() < sample_count) {
            this_thread::sleep_for(milliseconds(10));
        }
    } else {
        dds_entity_t ws = dds_create_waitset(participant);
        dds_waitset_attach(ws, dds_create_readcondition(reader, DDS_ANY_STATE), reader);

        while ((int)rtts.size() < sample_count) {
            dds_attach_t triggered[1];
            dds_return_t rc = dds_waitset_wait(ws, triggered, 1, DDS_MSECS(300));
            if (rc < 0) {
                cout << "Waitset failed" << endl;
                return -1;
            }
            rc = dds_take(reader, samples, infos, 1, 1);
            if (rc > 0 && infos[0].valid_data) {
                uint64_t recv_time = current_time_ns();
                uint64_t send_time;
                memcpy(&send_time, msg.payload._buffer, sizeof(uint64_t));
                rtts.push_back(recv_time - send_time);
            }
        }
    }

    if (!rtts.empty()) {
        if (logf.is_open()) {
            for (auto lat : rtts) {
                logf << frequency << "," << payload_size << "," << lat << endl;
            }
        }

        sort(rtts.begin(), rtts.end());
        auto get_percentile = [](const vector<uint64_t> &v, double p) {
            size_t idx = size_t(p * v.size());
            return v[min(idx, v.size() - 1)];
        };

        cout << "\nRTT stats (nanoseconds):" << endl;
        cout << "Min : " << rtts.front() << endl;
        cout << "p05 : " << get_percentile(rtts, 0.05) << endl;
        cout << "p50 : " << get_percentile(rtts, 0.50) << endl;
        cout << "p95 : " << get_percentile(rtts, 0.95) << endl;
        cout << "Max : " << rtts.back() << endl;
    }

    if (logf.is_open()) logf.close();
    dds_delete(participant);
    return 0;
}
