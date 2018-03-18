/**
 * @details Class to save patter point information
 */
class PatternPoint {
public:
    float x;
    float y;
    float radio;
    int h_father;
    PatternPoint() {
        x = 0;
        y = 0;
        radio = 0;
    }
    PatternPoint(float x, float y) {
        this->x = x;
        this->y = y;
    }
    PatternPoint(float x, float y, float radio, int h_father) {
        this->x = x;
        this->y = y;
        this->radio = radio;
        this->h_father = h_father;
    }
    float area(const PatternPoint p,const PatternPoint p2){
        return distance(p) * distance(p2);
    }
    float distance(const PatternPoint p) {
        return sqrt(pow(x - p.x, 2) + pow(y - p.y, 2));
    }
    Point2f to_point2f() {
        return Point2f(x, y);
    }
    Point2f center() {
        return Point2f(x, y);
    }
};