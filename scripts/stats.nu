#!/usr/bin/env nu

let csv_file = "./_results/2025-06-06-17:20:29/data.csv"

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
print $res
