import json
import random
from datetime import datetime, timedelta

labels_stereo = ["capasanta", "seabass", "mussel", "octopus", "clam"]
labels_mono = ["crab", "shrimp", "lobster", "squid", "eel"]

def random_float(min_val=0.0, max_val=100.0, precision=6):
    return round(random.uniform(min_val, max_val), precision)

def random_timestamp(start, end):
    delta = end - start
    random_seconds = random.randint(0, int(delta.total_seconds()))
    return (start + timedelta(seconds=random_seconds)).strftime("%Y-%m-%d %H:%M:%S")

def generate_entry(start_time, end_time):
    timestamp = random_timestamp(start_time, end_time)

    entry = [
        {
            "stereo_ir": {
                "timestamp": timestamp,
                "label": random.choice(labels_stereo)
            }
        },
        {
            "mono_ir": {
                "timestamp": timestamp,
                "label": random.choice(labels_mono),
                "confidence": f"{random_float(60.0, 100.0):.6f}",
                "box": [
                    f"{random_float()}",
                    f"{random_float()}",
                    f"{random_float()}",
                    f"{random_float()}"
                ]
            }
        },
        {
            "odometry": {
                "timestamp": timestamp,
                "traslation": [
                    f"{random_float()}",
                    f"{random_float()}",
                    f"{random_float()}"
                ],
                "rotation": [
                    f"{random_float()}",
                    f"{random_float()}",
                    f"{random_float()}",
                    f"{random_float()}",
                    f"{random_float()}",
                    f"{random_float()}",
                    f"{random_float()}",
                    f"{random_float()}",
                    f"{random_float()}"
                ]
            }
        }
    ]
    return entry

def main(output_file="generated_data.json", count=100):
    start_time = datetime.strptime("2000-00-00 00:00:00", "%Y-%m-%d %H:%M:%S")
    end_time = datetime.strptime("2100-00-00 00:00:00", "%Y-%m-%d %H:%M:%S")

    with open(output_file, "w") as f:
        for _ in range(count):
            entries = generate_entry(start_time, end_time)
            for e in entries:
                json.dump(e, f)
                f.write("\n")

    print(f"Generated {count * 3} entries with random timestamps in {output_file}")

if __name__ == "__main__":
    main()
