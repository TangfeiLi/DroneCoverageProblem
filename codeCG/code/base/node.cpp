
#include "node.h"

namespace drone_cover {

    bool Node::operator==(const Node &other) const {
        return (other.id == id);
    }

    std::ostream &operator<<(std::ostream &out, const Node &n) {
        out << "[" << n.id << ", " << n.loc << ", " << n.t <<", "<<n.drone_class->id;
        out << "]";
        return out;
    }
}
