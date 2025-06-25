#!/usr/bin/env nu


def cleanup [] {
    for kw in [
        one_way_latency
        sender
        receiver
    ] {
        try { pkill $kw }
    }
}

# let env_selection = {
#     "Express,CC_BLOCK": {
#         RUST_LOG: "z=error",
#         RECV: "./target/release/examples/receiver",
#         SEND: "./target/release/examples/sender --cc-block --express",
#     },
#     "Express": {
#         RUST_LOG: "z=error",
#         RECV: "./target/release/examples/receiver",
#         SEND: "./target/release/examples/sender --express",
#     },
#     "CC_BLOCK": {
#         RUST_LOG: "z=error",
#         RECV: "./target/release/examples/receiver",
#         SEND: "./target/release/examples/sender --cc-block",
#     },
#     "Default": {
#         RUST_LOG: "z=error",
#         RECV: "./target/release/examples/receiver",
#         SEND: "./target/release/examples/sender",
#     },
# }


let low_latency_config = {
    transport: {
        unicast: {
            lowlatency: true,
            qos: {
                enabled: false,
            }
        },
    }
}
let tx_config = {
    mode: "client",
    connect: { endpoints: ["udp/127.0.0.1:7447"] }
}
let rx_config = {
    mode: "peer",
    listen: { endpoints: $tx_config.connect.endpoints }
}

def make_tmp_config [cfg: record] {
    let file_path = mktemp --suffix .json
    $cfg | | to json | save -f $file_path
    return $file_path

}

let env_selection = {
    # "Express,CC_BLOCK": {
    #     RUST_LOG: "z=error",
    #     RECV: "./target/opt/examples/receiver",
    #     SEND: "./target/opt/examples/sender --cc-block --express",
    # },
    # "Express": {
    #     RUST_LOG: "z=error",
    #     RECV: "./target/opt/examples/receiver",
    #     SEND: "./target/opt/examples/sender --express",
    # },
    # "CC_BLOCK": {
    #     RUST_LOG: "z=error",
    #     RECV: "./target/opt/examples/receiver",
    #     SEND: "./target/opt/examples/sender --cc-block",
    # },
    # "Default": {
    #     RUST_LOG: "z=error",
    #     RECV: "./target/opt/examples/receiver",
    #     SEND: "./target/opt/examples/sender",
    # },
    # "Zenoh (express, ll)": {
    #     RUST_LOG: "z=error",
    #     # RECV: $"./target/opt/examples/receiver --config (make_tmp_config ($rx_config | merge $low_latency_config))",
    #     # SEND: $"./target/opt/examples/sender --express --config (make_tmp_config ($tx_config | merge $low_latency_config))",
    #     RECV: $"./target/opt/examples/receiver --config (make_tmp_config ($low_latency_config))",
    #     SEND: $"./target/opt/examples/sender --express --config (make_tmp_config ($low_latency_config))",
    #     # RECV: "./target/opt/examples/receiver --config ./_config-ll.json",
    #     # SEND: "./target/opt/examples/sender --express --config ./_config-ll.json",
    # },
    "Cyclone (listener)": {
        RECV: "./cyclone/build/receiver --listener",
        SEND: "./cyclone/build/sender",
    },
    "Cyclone (waitset)": {
        RECV: "./cyclone/build/receiver",
        SEND: "./cyclone/build/sender",
    },
    "Glommio": {
        RUST_LOG: "z=error",
        RECV: "./target/release/examples/one_way_latency --recv --mode glommio",
        SEND: "./target/release/examples/one_way_latency --mode glommio",
        # RECV: "./target/opt/examples/one_way_latency --recv --mode glommio",
        # SEND: "./target/opt/examples/one_way_latency --mode glommio",
    },
    "TokioTcp": {
        RUST_LOG: "z=error",
        RECV: "./target/release/examples/one_way_latency --recv --mode tokio",
        SEND: "./target/release/examples/one_way_latency --mode tokio",
        # RECV: "./target/opt/examples/one_way_latency --recv --mode tokio",
        # SEND: "./target/opt/examples/one_way_latency --mode tokio",
    },
    "Monoio": {
        RUST_LOG: "z=error",
        RECV: "./target/release/examples/one_way_latency --recv --mode monoio",
        SEND: "./target/release/examples/one_way_latency --mode monoio",
    },
    "TokioUdp": {
        RUST_LOG: "z=error",
        RECV: "./target/release/examples/one_way_latency --recv --mode udp",
        SEND: "./target/release/examples/one_way_latency --mode udp",
        # RECV: "./target/opt/examples/one_way_latency --recv --mode udp",
        # SEND: "./target/opt/examples/one_way_latency --mode udp",
    },
    "Zenoh (exp)": {
        RUST_LOG: "z=error",
        RECV: "./target/release/examples/one_way_latency --recv",
        SEND: "./target/release/examples/one_way_latency --express",
    },
    "Zenoh (low-lat)": {
        RUST_LOG: "z=error",
        RECV: "./target/release/examples/one_way_latency --recv --config (make_tmp_config ($low_latency_config))",
        SEND: "./target/release/examples/one_way_latency --config (make_tmp_config ($low_latency_config))",
    },
    # "Foo (express)": {
    #     RUST_LOG: "z=error",
    #     RECV: "./target/release/examples/one_way_latency --recv --mode foo",
    #     SEND: "./target/release/examples/one_way_latency --express --mode foo",
    #     # RECV: "./target/opt/examples/one_way_latency --recv",
    #     # SEND: "./target/opt/examples/one_way_latency --express",
    # },
    "Zenoh": {
        RUST_LOG: "z=error",
        RECV: "./target/release/examples/one_way_latency --recv",
        SEND: "./target/release/examples/one_way_latency",
        # RECV: "./target/opt/examples/one_way_latency --recv",
        # SEND: "./target/opt/examples/one_way_latency",
    },
    "StdUdp": {
        RECV: "./target/release/examples/one_way_latency --recv --mode std_udp",
        SEND: "./target/release/examples/one_way_latency --mode std_udp",
    },
    "StdTcp": {
        RECV: "./target/release/examples/one_way_latency --recv --mode std_tcp",
        SEND: "./target/release/examples/one_way_latency --mode std_tcp",
    },
}


# let out_dir = "_results" | path join (date now | format date "%Y-%m-%d-%T")
# mkdir -v $out_dir
let out_dir = "_results/tmp"
rm -rf $out_dir
mkdir -v $out_dir

let log_file = $out_dir | path join "log.txt"
let err_file = $out_dir | path join "err.txt"

for payload in [64] {
# for payload in [64] {
# for payload in [(32 * 1024)] {
    # for freq in [0] {
    for freq in [0, 10, 100, 1000, 10000] {
    # for freq in [10, 100] {
    # for freq in [0, 10, 100] {
    # for freq in [10, 100, 1000] {
    # for freq in [10] {
# for payload in [64, (64 * 1024)] {
#     for freq in [0, 10, 1000, 10000] {
        let sample = if $freq == 0 {
            200000
        } else {
            # $freq * 5
            # $freq * 5
            $freq * 20
            # $freq * 60
        }

        let res = $env_selection
        | transpose name local_env
        | reverse
        | each {|x|
            with-env $x.local_env {
                let name_dir = $out_dir | path join $x.name
                mkdir $name_dir
                let recv = {
                    let csv_file = $name_dir | path join $"($freq)-($payload).csv"
                    (
                        # perf stat
                        taskset -c 0,2
                        ...($env.RECV | split row " ")
                        --frequency $freq
                        --sample $sample
                        --payload $payload
                        --log $csv_file
                        --warmup 1
                    )
                }
                let send = {
                    sleep 1sec
                    let sender_log_file = $name_dir | path join $"($freq)-($payload)-sender.log"
                    (
                        taskset -c 1,3
                        ...($env.SEND | split row " ")
                        --frequency $freq
                        --sample 0
                        --payload $payload
                        o+e> $sender_log_file
                    )
                }

                let txt = $"\n\n>>> Begin testing with ($x.name)"
                print $txt
                echo $txt | save -a $log_file
                echo "\n" | save -a $log_file
                let send_id = job spawn $send
                do $recv | tee { save -a $log_file --stderr $err_file }
                job kill $send_id
                cleanup
            }
        }
        sleep 1sec
    }
}

ls ($out_dir | path join "*/*.csv" | into glob)
| get name
| each {|file|
    let name = $file | path split | reverse | get 1
    open $file | insert "Name" $name
}
| flatten
| each {|row|
    $row | update "Latency" (($row.Latency / 1000) | into int)
}
| save -f ($out_dir | path join "data.csv")


let csv_file = $out_dir | path join "data.csv"
let res = open $csv_file
| group-by Frequency Payload Name --to-table
| each {|g|
    let sorted = $g.items | get Latency | sort
    let count = $sorted | length
    let min = $sorted | first
    let p05 = $sorted | get (($count * 0.05) | into int)
    let p50 = $sorted | get (($count * 0.50) | into int)
    let p95 = $sorted | get (($count * 0.95) | into int)
    let max = $sorted | last
    let row = {
        Frequency: $g.Frequency,
        Payload: $g.Payload,
        Name: $g.Name,
        Min: $min,
        p05: $p05,
        p50: $p50,
        p95: $p95,
        max: $max,
    }
    $row
}
| sort-by Frequency Payload p50 p95 max p05

$res | save -f ($out_dir | path join "stats.csv")
print $res

rm -f tmp.*.json
