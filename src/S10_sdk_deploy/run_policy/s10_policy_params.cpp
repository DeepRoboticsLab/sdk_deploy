#include <array>

struct S10PolicyParams {
    std::array<float, 16> kp;
    std::array<float, 16> kd;
};

const S10PolicyParams& GetS10PolicyParams() {
    static const S10PolicyParams kParams{
        {
            80.0f, 80.0f, 80.0f, 0.0f,
            80.0f, 80.0f, 80.0f, 0.0f,
            80.0f, 80.0f, 80.0f, 0.0f,
            80.0f, 80.0f, 80.0f, 0.0f
        },
        {
            2.0f, 2.0f, 2.0f, 0.8f,
            2.0f, 2.0f, 2.0f, 0.8f,
            2.0f, 2.0f, 2.0f, 0.8f,
            2.0f, 2.0f, 2.0f, 0.8f
        }
    };
    return kParams;
}
