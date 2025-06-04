#include "FileReader.h"

#include <fstream>
#include <sstream>
#include <iomanip>      // std::get_time
#include <ctime>        // std::tm, std::time_t
#include <nlohmann/json.hpp>

using namespace sensingrigs_communicator::msg;
using builtin_interfaces::msg::Time;
using json = nlohmann::json;


FileReader::FileReader(const std::string& file_path)
  : _path(file_path), _valid(false)
{
    // Il costruttore si limita a memorizzare il path del file JSON; non esegue I/O.
}

bool FileReader::ok() const noexcept
{
    return _valid;
    // Se load() viene eseguito ritrona true
}

// Getters per vettori di dati json
const std::vector<StereoIR>& FileReader::getStereoData() const noexcept
{
    return _stereo_vec;
}

const std::vector<MonoIR>& FileReader::getMonoData() const noexcept
{
    return _mono_vec;
}

const std::vector<Odometry>& FileReader::getOdometryData() const noexcept
{
    return _odom_vec;
}

void FileReader::load()
{
    // 1) Legge tutto il JSON in std::string
    std::string file_contents = readFileToString();

    // 2) Fai il parsing JSON
    json root;
    try {
        root = json::parse(file_contents);
    }
    catch (const json::parse_error& e) {
        throw std::runtime_error(
            "FileReader::load() - parsing JSON fallito in '" + _path + "': " + std::string(e.what())
        );
    }

    // 3) Verifica che “data” esista ed è un array
    // rischio eccezione se non controllato
    if (!root.contains("data") || !root["data"].is_array()) {
        throw std::runtime_error(
            "FileReader::load() - manca il campo 'data' o non è array in '" + _path + "'"
        );
    }

    // 4) Smista ogni elemento di root["data"], che equivale ad un array contenente tutto ci; che sta dentro data nel json
    parseRoot(root);

    // 5) Se si arriva qui, tutto è andato a buon fine
    _valid = true;
}

std::string FileReader::readFileToString() const
{
    std::ifstream ifs(_path, std::ios::in | std::ios::binary);
    if (!ifs.is_open()) {
        throw std::runtime_error("FileReader::readFileToString() - impossibile aprire '" + _path + "'");
    }
    std::ostringstream oss;
    oss << ifs.rdbuf();
    return oss.str();
}

void FileReader::parseRoot(const json& root)
{
    const auto& arr = root["data"];

    // Puliamo eventuali dati precedenti (nel caso di doppio load)
    _stereo_vec.clear();
    _mono_vec.clear();
    _odom_vec.clear();

    // Iteriamo su ciascun elemento dell’array
    for (size_t i = 0; i < arr.size(); ++i)
    {
        const json& element = arr[i];

        // Ogni elemento dev’essere un oggetto con una sola chiave:
        //   { "stereo_ir": { … } } oppure { "mono_ir": { … } } oppure { "odometry": { … } }
        if (!element.is_object() || element.size() != 1) {
            throw std::runtime_error(
                "FileReader::parseRoot() - elemento data[" + std::to_string(i) +
                "] non è un oggetto singolo con unica chiave."
            );
        }

        // Prendiamo la chiave (e il relativo value)
        const std::string key = element.begin().key();   // es. "stereo_ir"
        const json&        body = element.begin().value();

        if (key == "stereo_ir") {
            parseStereo(body);
        }
        else if (key == "mono_ir") {
            parseMono(body);
        }
        else if (key == "odometry") {
            parseOdom(body);
        }
        else {
            throw std::runtime_error(
                "FileReader::parseRoot() - chiave sconosciuta '" + key +
                "' in data[" + std::to_string(i) + "]."
            );
        }
    }
}

void FileReader::parseStereo(const json& stereo_node)
{
    // Controlli preliminari
    if (!stereo_node.is_object()) {
        throw std::runtime_error("FileReader::parseStereo() - stereo_ir non è un oggetto JSON.");
    }
    // Campi obbligatori: "timestamp" (string), "label" (string)
    if (!stereo_node.contains("timestamp") || !stereo_node.contains("label")) {
        throw std::runtime_error(
            "FileReader::parseStereo() - manca 'timestamp' o 'label' in stereo_ir."
        );
    }
    if (!stereo_node["timestamp"].is_string()) {
        throw std::runtime_error("FileReader::parseStereo() - 'timestamp' non è string.");
    }
    if (!stereo_node["label"].is_string()) {
        throw std::runtime_error("FileReader::parseStereo() - 'label' non è string.");
    }

    // Popoliamo il messaggio ROS2
    StereoIR msg;
    // 1) timestamp: converti string -> builtin_interfaces::msg::Time
    msg.timestamp = parseTimestamp(stereo_node["timestamp"].get<std::string>());

    // 2) label (string<=10)
    std::string label = stereo_node["label"].get<std::string>();
    if (label.size() > 10) {
        throw std::runtime_error("FileReader::parseStereo() - 'label' troppo lungo (>10).");
    }
    msg.label = std::move(label);

    // Aggiungiamo al vettore
    _stereo_vec.push_back(std::move(msg));
}

void FileReader::parseMono(const json& mono_node)
{
    // Controlli preliminari
    if (!mono_node.is_object()) {
        throw std::runtime_error("FileReader::parseMono() - mono_ir non è un oggetto JSON.");
    }
    // Campi obbligatori: "timestamp" (string), "label" (string),
    //                  "confidence" (number), "box" (array di 4 numeri)
    if (!mono_node.contains("timestamp") ||
        !mono_node.contains("label") ||
        !mono_node.contains("confidence") ||
        !mono_node.contains("box"))
    {
        throw std::runtime_error(
            "FileReader::parseMono() - manca almeno uno tra 'timestamp', 'label', 'confidence', 'box' in mono_ir."
        );
    }

    // Verifica tipi
    if (!mono_node["timestamp"].is_string()) {
        throw std::runtime_error("FileReader::parseMono() - 'timestamp' non è string.");
    }
    if (!mono_node["label"].is_string()) {
        throw std::runtime_error("FileReader::parseMono() - 'label' non è string.");
    }
    if (!mono_node["confidence"].is_number()) {
        throw std::runtime_error("FileReader::parseMono() - 'confidence' non è number.");
    }
    if (!mono_node["box"].is_array() || mono_node["box"].size() != 4) {
        throw std::runtime_error("FileReader::parseMono() - 'box' non è array di 4 elementi.");
    }

    // Popoliamo il messaggio ROS2
    MonoIR msg;
    // 1) timestamp
    msg.timestamp = parseTimestamp(mono_node["timestamp"].get<std::string>());

    // 2) label (max 10 char)
    std::string label = mono_node["label"].get<std::string>();
    if (label.size() > 10) {
        throw std::runtime_error("FileReader::parseMono() - 'label' troppo lungo (>10).");
    }
    msg.label = std::move(label);

    // 3) confidence
    msg.confidence = static_cast<float>(mono_node["confidence"].get<double>());

    // 4) box (array di 4 float)
    for (size_t i = 0; i < 4; ++i) {
        const auto& v = mono_node["box"][i];
        if (!v.is_number()) {
            throw std::runtime_error("FileReader::parseMono() - elemento di 'box' non è number.");
        }
        msg.box[i] = static_cast<float>(v.get<double>());
    }

    _mono_vec.push_back(std::move(msg));
}

void FileReader::parseOdom(const json& odom_node)
{
    // Controlli preliminari
    if (!odom_node.is_object()) {
        throw std::runtime_error("FileReader::parseOdom() - odometry non è un oggetto JSON.");
    }
    // Campi obbligatori: "timestamp" (string), "translation" (array di 3), "rotation" (array di 9)
    if (!odom_node.contains("timestamp") ||
        !odom_node.contains("translation") ||
        !odom_node.contains("rotation"))
    {
        throw std::runtime_error(
            "FileReader::parseOdom() - manca almeno uno tra 'timestamp', 'translation', 'rotation'."
        );
    }

    if (!odom_node["timestamp"].is_string()) {
        throw std::runtime_error("FileReader::parseOdom() - 'timestamp' non è string.");
    }
    if (!odom_node["translation"].is_array() || odom_node["translation"].size() != 3) {
        throw std::runtime_error("FileReader::parseOdom() - 'translation' non è array di 3.");
    }
    if (!odom_node["rotation"].is_array() || odom_node["rotation"].size() != 9) {
        throw std::runtime_error("FileReader::parseOdom() - 'rotation' non è array di 9.");
    }

    // Popoliamo il messaggio ROS2
    Odometry msg;
    // 1) timestamp
    msg.timestamp = parseTimestamp(odom_node["timestamp"].get<std::string>());

    // 2) translation (array di 3 double)
    for (size_t i = 0; i < 3; ++i) {
        const auto& v = odom_node["translation"][i];
        if (!v.is_number()) {
            throw std::runtime_error("FileReader::parseOdom() - elemento di 'translation' non è number.");
        }
        msg.translation[i] = v.get<double>();
    }

    // 3) rotation (array di 9 double)
    for (size_t i = 0; i < 9; ++i) {
        const auto& v = odom_node["rotation"][i];
        if (!v.is_number()) {
            throw std::runtime_error("FileReader::parseOdom() - elemento di 'rotation' non è number.");
        }
        msg.rotation[i] = v.get<double>();
    }

    _odom_vec.push_back(std::move(msg));
}

// Questa funzione serve per trasformare il timestamp da stringa a Time, come richiesto nei .msg

builtin_interfaces::msg::Time FileReader::parseTimestamp(const std::string& timestr) const
{
    // Ci aspettiamo il formato "YYYY-MM-DD HH:MM:SS"
    // Creiamo uno std::tm e facciamo std::get_time:
    std::tm tm = {};
    std::istringstream iss(timestr);
    // Nota: %Y=%4-digit year, %m=%02-digit month, %d=%02-digit day,
    //       %H=%02-digit hour (24h), %M=%02-digit minute, %S=%02-digit second
    iss >> std::get_time(&tm, "%Y-%m-%d %H:%M:%S");
    if (iss.fail()) {
        throw std::runtime_error(
            "FileReader::parseTimestamp() - fallito parsing di \"" + timestr +
            "\" con formato YYYY-MM-DD HH:MM:SS."
        );
    }

    // Convertiamo tm in time_t (assumiamo UTC, quindi usiamo timegm o _mkgmtime su Windows)
    // Purtroppo std::mktime assume tm in locale, quindi per UTC c'è la funzione timegm su POSIX.
    // Se non è disponibile, si può adattare (ma qui assumiamo POSIX).
    #if defined(_WIN32)
        std::time_t tt = _mkgmtime(&tm); // su Windows: _mkgmtime è equivalente di timegm
    #else
        std::time_t tt = timegm(&tm);    // POSIX: timegm() assume tm come UTC
    #endif

    if (tt == static_cast<std::time_t>(-1)) {
        throw std::runtime_error(
            "FileReader::parseTimestamp() - timegm/mkgmtime restituisce -1 per \"" + timestr + "\""
        );
    }

    // Ora costruiamo builtin_interfaces::msg::Time
    builtin_interfaces::msg::Time ros_time;
    ros_time.sec = static_cast<int32_t>(tt);
    ros_time.nanosec = 0u;
    return ros_time;
}
