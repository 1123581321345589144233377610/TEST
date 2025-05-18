#include <iostream>
#include <fstream>
#include <vector>
#include <cstdint>
#include <iomanip>
#include <algorithm>
#include <string>

#pragma pack(push, 1)
struct TargetInfo {
    uint8_t target_id;       
    int16_t vertical_distance;
    int16_t lateral_distance; 
    int16_t speed_y;          
    uint8_t target_type;      
    uint8_t lane_number;      
    int16_t front_spacing;    
    int16_t front_time_interval;       
    int16_t speed_x;          
    int16_t heading_angle;  
    uint8_t events;         
    int32_t radar_x;          
    int32_t radar_y;         
    uint8_t blind_mark;       
    uint8_t car_length;       
    uint8_t car_width;        
};
#pragma pack(pop)

std::string get_target_type(uint8_t type) {
    switch(type) {
        case 0: return "Small car";
        case 1: return "Pedestrian";
        case 2: return "Non-motor vehicles";
        case 3: return "Medium-sized car";
        case 4: return "Large car";
        default: return "Unknown (" + std::to_string(type) + ")";
    }
}

std::vector<uint8_t> read_file(const std::string& filename) {
    std::ifstream file(filename, std::ios::binary | std::ios::ate);
    if (!file) throw std::runtime_error("Cannot open file");
    size_t size = file.tellg();
    file.seekg(0);
    std::vector<uint8_t> buffer(size);
    file.read(reinterpret_cast<char*>(buffer.data()), size);
    return buffer;
}

void parse_4d42_block(const uint8_t* data, size_t size, std::ostream& output_stream) {
    if (size < 4) throw std::invalid_argument("Invalid block size");
    
    if (data[0] != 0x4D || data[1] != 0x42) 
        throw std::invalid_argument("Not a 4D42 block");
    
    uint16_t data_length = (data[3] << 8) | data[2];
    if (data_length < 2 || data_length > size - 4) 
        throw std::invalid_argument("Invalid data length");
    
    const uint8_t* payload = data + 4;
    size_t payload_size = data_length - 2;
    
    size_t num_targets = payload_size / sizeof(TargetInfo);
    if (num_targets > 128) {
        output_stream << "Warning: Invalid number of targets (" << num_targets << "), truncating to 128\n";
        num_targets = 128;
    }
    
    const TargetInfo* targets = reinterpret_cast<const TargetInfo*>(payload);
    
    output_stream << "Found " << num_targets << " targets:\n";
    for (size_t i = 0; i < num_targets; ++i) {
        const auto& t = targets[i];
        output_stream << "Target " << static_cast<int>(t.target_id) << ":\n"
                  << "  Vertical: " << t.vertical_distance / 10.0 << " m\n"
                  << "  Lateral: " << t.lateral_distance / 10.0 << " m\n"
                  << "  Speed Y: " << t.speed_y / 10.0 << " m/s\n"
                  << "  Type: " << get_target_type(t.target_type) << "\n"
                  << "  Lane: ";
        
        // Проверка корректности номера 
        if (t.lane_number >= 1 && t.lane_number <= 8) {
            output_stream << static_cast<int>(t.lane_number);
        } else {
            output_stream << "Invalid (" << static_cast<int>(t.lane_number) << ")";
        }
        
        output_stream << "\n  Front space: " << t.front_spacing / 10.0 << " m\n"
                  << "  Front time: " << t.front_time_interval / 10.0 << " s\n"
                  << "  Speed X: " << t.speed_x / 10.0 << " m/s\n"
                  << "  Heading: " << t.heading_angle / 10.0 << " deg\n"
                  << "  Events: 0x" << std::hex << std::setw(2) << std::setfill('0') 
                  << static_cast<int>(t.events) << "\n"
                  << std::dec << "----------------\n";
    }
}

void find_and_parse_4d42(const std::vector<uint8_t>& data, std::ostream& output_stream) {
    const uint8_t pattern[] = {0x4D, 0x42};
    auto it = std::search(data.begin(), data.end(), 
                         std::begin(pattern), std::end(pattern));
    
    if (it == data.end()) {
        output_stream << "4D42 block not found\n";
        return;
    }
    
    size_t offset = std::distance(data.begin(), it);
    if (data.size() - offset < 4) {
        output_stream << "Incomplete 4D42 header\n";
        return;
    }
    
    try {
        parse_4d42_block(&data[offset], data.size() - offset, output_stream);
    } catch (const std::exception& e) {
        output_stream << "Error parsing block: " << e.what() << "\n";
    }
}

int main() {
    try {
        auto buffer = read_file("b4a2a978-fd42-489b-bc7d-c0eaffef9311.bin");
        std::ofstream output_file("output.txt");
        if (!output_file) throw std::runtime_error("Cannot create output file");
        find_and_parse_4d42(buffer, output_file);
        std::cout << "Results saved to output.txt\n";
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
    return 0;
}