#ifndef FILEREADER_H
#define FILEREADER_H

#include <string>
#include <vector>
#include <array>
#include <stdexcept>

// Includi i messaggi generati da ROS2
#include <../build/sensingrigs_communicator/rosidl_generator_cpp/sensingrigs_communicator/msg/stereo_ir.hpp>
#include <../build/sensingrigs_communicator/rosidl_generator_cpp/sensingrigs_communicator/msg/mono_ir.hpp>
#include <../build/sensingrigs_communicator/rosidl_generator_cpp/sensingrigs_communicator/msg/odometry.hpp>

// Includi il time di ROS2 (builtin_interfaces/Time)
#include <builtin_interfaces/msg/time.hpp>

// JSON (nlohmann)
#include <nlohmann/json.hpp>
using json = nlohmann::json;

/**
 * @brief FileReader: legge un JSON “standardizzato” come example.json e popola
 *        tre array di messaggi ROS2 (StereoIR, MonoIR, Odometry).
 */
class FileReader
{
public:
    /**
     * @param file_path  Path al file JSON (es. "docs_file/example.json").
     */
    explicit FileReader(const std::string& file_path);

    /**
     * @brief Legge e parsea il JSON; popola _stereo_vec, _mono_vec e _odom_vec.
     *        Se il file non esiste, il JSON è malformato, o mancano campi, lancia
     *        std::runtime_error con messaggio descrittivo.
     */
    void load();

    /**
     * @return true se load() è terminato con successo (senza eccezioni).
     */
    bool ok() const noexcept;

    /**
     * @return const reference al vettore di StereoIR parsati.
     */
    const std::vector<sensingrigs_communicator::msg::StereoIR>&
    getStereoData() const noexcept;

    /**
     * @return const reference al vettore di MonoIR parsati.
     */
    const std::vector<sensingrigs_communicator::msg::MonoIR>&
    getMonoData() const noexcept;

    /**
     * @return const reference al vettore di Odometry parsati.
     */
    const std::vector<sensingrigs_communicator::msg::Odometry>&
    getOdometryData() const noexcept;

private:
    std::string _path;
    bool        _valid{false};

    // I tre container interni
    std::vector<sensingrigs_communicator::msg::StereoIR> _stereo_vec;
    std::vector<sensingrigs_communicator::msg::MonoIR>   _mono_vec;
    std::vector<sensingrigs_communicator::msg::Odometry> _odom_vec;

    /**
     * @brief Legge l’intero file in una stringa.
     * @throws std::runtime_error se non riesce ad aprire il file.
     */
    std::string readFileToString() const;

    /**
     * @brief Dato il JSON radice, itera su root["data"] e smista ciascun elemento
     *        a parseStereo(), parseMono() o parseOdom().
     * @throws std::runtime_error se root non ha “data” oppure se la struttura non è corretta.
     */
    void parseRoot(const json& root);

    /**
     * @brief Parsing di singolo oggetto “stereo_ir” (contenente solo timestamp e label).
     * @throws std::runtime_error se mancano campi o i tipi sono sbagliati.
     */
    void parseStereo(const json& stereo_node);

    /**
     * @brief Parsing di singolo oggetto “mono_ir” (timestamp, label, confidence, box).
     * @throws std::runtime_error se mancano campi o i tipi sono sbagliati.
     */
    void parseMono(const json& mono_node);

    /**
     * @brief Parsing di singolo oggetto “odometry” (timestamp, translation, rotation).
     * @throws std::runtime_error se mancano campi o i tipi sono sbagliati.
     */
    void parseOdom(const json& odom_node);

    /**
     * @brief Converte una stringa “YYYY-MM-DD HH:MM:SS” in builtin_interfaces::msg::Time.
     * @throws std::runtime_error se la stringa non rispetta il formato o la conversione fallisce.
     */
    builtin_interfaces::msg::Time parseTimestamp(const std::string& timestr) const;
};

#endif