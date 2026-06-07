import sys
import os

def analyze_log(file_path):
    if not os.path.exists(file_path):
        print(f"Error: Log file '{file_path}' not found.")
        return

    with open(file_path, 'r', encoding='utf-8') as f:
        lines = f.readlines()

    total_valid_frames = 0
    hits = 0

    for i, line in enumerate(lines):
        line = line.strip()
        
        # 1. Extract the computed steering angle
        if line.startswith("group_21;"):
            parts = line.split(";")
            if len(parts) < 3:
                continue
            try:
                computed_angle = float(parts[2])
            except ValueError:
                continue

            # 2. Find the corresponding ground truth angle in the following lines
            ground_truth = None
            for j in range(i + 1, min(i + 12, len(lines))):
                next_line = lines[j].strip()
                if next_line.startswith("main: groundSteering ="):
                    try:
                        ground_truth = float(next_line.split("=")[1].strip())
                    except (IndexError, ValueError):
                        pass
                    break

            # 3. Only evaluate frames where ground truth is not 0.0
            if ground_truth is not None and ground_truth != 0.0:
                total_valid_frames += 1
                
                # Check if the absolute error is within the allowed range
                if abs(computed_angle - ground_truth) <= 0.09:
                    hits += 1

    # 4. Print evaluation report
    print("=" * 50)
    print("     OPENDLV ALGORITHM EVALUATION REPORT      ")
    print("=" * 50)
    print(f" Target Log File : {file_path}")
    print(f" Valid Frames    : {total_valid_frames}")
    print(f" Successful Hits : {hits}")
    print("-" * 50)
    
    if total_valid_frames > 0:
        accuracy_rate = (hits / total_valid_frames) * 100
        print(f" Accuracy Rate   : {accuracy_rate:.2f}%")
        
        if accuracy_rate >= 35.0:
            print(" Result          : PASSED. Threshold met. Ready for delivery.")
        else:
            print(" Result          : FAILED. Accuracy below 35%. Tuning required.")
    else:
        print(" Warning         : No valid 'groundSteering' data found in log.")
    print("=" * 50)

if __name__ == "__main__":
    target_file = "drive_test.log"
    if len(sys.argv) > 1:
        target_file = sys.argv[1]
    analyze_log(target_file)
