import statistics

def extract_values(filename):
    data = {
        "Positive Slope Left": [],
        "Positive Slope Right": [],
        "Positive Intercept Left": [],
        "Positive Intercept Right": [],
        "Negative Slope Left": [],
        "Negative Slope Right": [],
        "Negative Intercept Left": [],
        "Negative Intercept Right": []
    }

    with open(filename, 'r') as file:
        lines = file.readlines()

    for line in lines:
        line = line.strip()
        if line.startswith('Positive Slope:'):
            left, right = line.split(':')[1].split()
            data["Positive Slope Left"].append(float(left))
            data["Positive Slope Right"].append(float(right))
        elif line.startswith('Positive Intercept:'):
            left, right = line.split(':')[1].split()
            data["Positive Intercept Left"].append(float(left))
            data["Positive Intercept Right"].append(float(right))
        elif line.startswith('Negative Slope:'):
            left, right = line.split(':')[1].split()
            data["Negative Slope Left"].append(float(left))
            data["Negative Slope Right"].append(float(right))
        elif line.startswith('Negative Intercept:'):
            left, right = line.split(':')[1].split()
            data["Negative Intercept Left"].append(float(left))
            data["Negative Intercept Right"].append(float(right))

    return data

def calculate_statistics(data):
    stats = {}
    for key, values in data.items():
        stats[key] = {
            "mean": statistics.mean(values),
            "variance": statistics.variance(values)
        }
    return stats

# Replace 'your_file.txt' with the path to your text file
filename = '1_1.txt'
data = extract_values(filename)
stats = calculate_statistics(data)

for key, value in stats.items():
    print(f"{key}: Mean = {value['mean']}, Variance = {value['variance']}")
