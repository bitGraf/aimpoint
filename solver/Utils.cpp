#include "Utils.h"

namespace ab_solver {
    namespace utils {
        
        void freeArray(double* ptr) {
            delete[] ptr;
            ptr = nullptr;
        }
    }
}