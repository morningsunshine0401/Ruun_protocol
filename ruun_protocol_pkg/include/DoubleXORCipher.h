#include <vector>
#include <cstdint>
#include <cstring>

class DoubleXORCipher {
public:
  DoubleXORCipher(const std::vector<double>& key) : key(key) {}

  void encrypt(std::vector<double>& data) {
    for (size_t i = 0; i < data.size(); ++i) {
      data[i] = xorDouble(data[i], key[i % key.size()]);
    }
  }

  void decrypt(std::vector<double>& data) {
    encrypt(data); // Encryption and decryption are the same operation in XOR cipher
  }

private:
  std::vector<double> key;

  double xorDouble(double a, double b) {
    uint64_t int_a, int_b, int_result;
    memcpy(&int_a, &a, sizeof(a));
    memcpy(&int_b, &b, sizeof(b));
    int_result = int_a ^ int_b;
    double result;
    memcpy(&result, &int_result, sizeof(result));
    return result;
  }
};
