#!/usr/bin/env nu


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

let env_selection = {
    "Express,CC_BLOCK": {
        RUST_LOG: "z=error",
        RECV: "./target/opt/examples/receiver",
        SEND: "./target/opt/examples/sender --cc-block --express",
    },
    "Express": {
        RUST_LOG: "z=error",
        RECV: "./target/opt/examples/receiver",
        SEND: "./target/opt/examples/sender --express",
    },
    "CC_BLOCK": {
        RUST_LOG: "z=error",
        RECV: "./target/opt/examples/receiver",
        SEND: "./target/opt/examples/sender --cc-block",
    },
    "Default": {
        RUST_LOG: "z=error",
        RECV: "./target/opt/examples/receiver",
        SEND: "./target/opt/examples/sender",
    },
}


let out_dir = "_results" | path join (date now | format date "%Y-%m-%d-%T")
mkdir -v $out_dir
let log_file = $out_dir | path join "log.txt"
let err_file = $out_dir | path join "err.txt"

# for payload in [64, 65536] {
#     for freq in [0] {
for payload in [64, (64 * 1024)] {
    for freq in [0, 10, 1000, 10000] {
        let sample = if $freq == 0 {
            100000
        } else {
            $freq * 10
        }

        let res = $env_selection
        | transpose name local_env
        | reverse
        | each {|x|
            with-env $x.local_env {
                let recv = {
                    let name_dir = $out_dir | path join $x.name
                    mkdir $name_dir
                    let csv_file = $name_dir | path join $"($freq)-($payload).csv"
                    taskset -c 0,2 ...($env.RECV | split row " ") -f $freq -s $sample -p $payload -l $csv_file --warmup 2
                }
                let send = {
                    # sleep 1sec
                    taskset -c 1,3 ...($env.SEND | split row " ") -f $freq -s 0 -p $payload
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
    $row | update "RTT" (($row.RTT / 1000) | into int)
}
| save -f ($out_dir | path join "data.csv")


let csv_file = $out_dir | path join "data.csv"
let res = open $csv_file
| group-by Frequency Payload Name --to-table
| each {|g|
    let sorted = $g.items | get RTT | sort
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
| sort-by Payload p50 p95 p05

$res | save -f ($out_dir | path join "stats.csv")
print $res
