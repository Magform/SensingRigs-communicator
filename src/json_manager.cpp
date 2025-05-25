#include <iostream>
#include <fstream>
#include <string>
//sudo apt-get install nlohmann-json3-dev ,per installare la libreria seguente:
#include <nlohmann/json.hpp>

using json = nlohmann::json;

void print_stereo_ir(const json& data, std::ofstream& out_file) {
    std::cout << "Stereo IR Data:\n";
    out_file << "Stereo IR Data:\n";
    
    std::cout << "  Timestamp: " << data["timestamp"].get<std::string>() << "\n";
    out_file << "  Timestamp: " << data["timestamp"].get<std::string>() << "\n";
    
    std::cout << "  Label: " << data["label"].get<std::string>() << "\n\n";
    out_file << "  Label: " << data["label"].get<std::string>() << "\n\n";
}

void print_mono_ir(const json& data, std::ofstream& out_file) {
    std::cout << "Mono IR Data:\n";
    out_file << "Mono IR Data:\n";
    
    std::cout << "  Timestamp: " << data["timestamp"].get<std::string>() << "\n";
    out_file << "  Timestamp: " << data["timestamp"].get<std::string>() << "\n";
    
    std::cout << "  Label: " << data["label"].get<std::string>() << "\n";
    out_file << "  Label: " << data["label"].get<std::string>() << "\n";
    
    std::cout << "  Confidence: " << data["confidence"].get<double>() << "\n";
    out_file << "  Confidence: " << data["confidence"].get<double>() << "\n";
    
    std::cout << "  Bounding Box: [";
    out_file << "  Bounding Box: [";
    for (const auto& coord : data["box"]) {
        std::cout << coord.get<double>() << ", ";
        out_file << coord.get<double>() << ", ";
    }
    std::cout << "]\n\n";
    out_file << "]\n\n";
}

void print_odometry(const json& data, std::ofstream& out_file) {
    std::cout << "Odometry Data:\n";
    out_file << "Odometry Data:\n";
    
    std::cout << "  Timestamp: " << data["timestamp"].get<std::string>() << "\n";
    out_file << "  Timestamp: " << data["timestamp"].get<std::string>() << "\n";
    
    std::cout << "  Translation: [";
    out_file << "  Translation: [";
    for (const auto& val : data["translation"]) {
        std::cout << val.get<double>() << " ";
        out_file << val.get<double>() << " ";
    }
    std::cout << "]\n";
    out_file << "]\n";
    
    std::cout << "  Rotation Matrix: [";
    out_file << "  Rotation Matrix: [";
    for (const auto& val : data["rotation"]) {
        std::cout << val.get<double>() << " ";
        out_file << val.get<double>() << " ";
    }
    std::cout << "]\n\n";
    out_file << "]\n\n";
}

int main() {
    try {
        std::ifstream input_file("example.json");
        if (!input_file.is_open()) {
            std::cerr << "Error opening input file!\n";
            return 1;
        }

        std::ofstream out_file("MonoIR.msg");
        if (!out_file.is_open()) {
            std::cerr << "Error opening output file!\n";
            return 1;
        }

        // Try to parse as single JSON first
        try {
            json data;
            input_file >> data;
            
            if (data.contains("data")) {
                // Array format
                for (const auto& item : data["data"]) {
                    if (item.contains("stereo_ir")) print_stereo_ir(item["stereo_ir"], out_file);
                    else if (item.contains("mono_ir")) print_mono_ir(item["mono_ir"], out_file);
                    else if (item.contains("odometry")) print_odometry(item["odometry"], out_file);
                }
                return 0;
            }
        } catch (...) {
            // If single JSON parse fails, try JSON lines format
            input_file.clear();
            input_file.seekg(0);
        }

        // JSON lines format
        std::string line;
        while (std::getline(input_file, line)) {
            if (line.empty()) continue;
            
            try {
                json data = json::parse(line);
                
                if (data.contains("stereo_ir")) print_stereo_ir(data["stereo_ir"], out_file);
                else if (data.contains("mono_ir")) print_mono_ir(data["mono_ir"], out_file);
                else if (data.contains("odometry")) print_odometry(data["odometry"], out_file);
            } catch (const json::parse_error& e) {
                std::cerr << "JSON parse error: " << e.what() << "\n";
                out_file << "JSON parse error: " << e.what() << "\n";
            }
        }
        
        input_file.close();
        out_file.close();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }

    return 0;
}