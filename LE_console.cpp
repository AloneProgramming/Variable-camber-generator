#include <iostream>
#include <cmath>
#include <vector>
#include <iomanip>
#include <algorithm> 

using namespace std;

double calculate_yc(double x, double m, double p) {
    if (p == 0 || p == 1) {
        return 0;
    }
    if (x <= p) {
        return (m / pow(p, 2)) * (2 * p * x - pow(x, 2));
    } else {
        return (m / pow(1 - p, 2)) * ((1 - 2 * p) + 2 * p * x - pow(x, 2));
    }
}

double calculate_dyc_dx(double x, double m, double p) {
    if (p == 0 || p == 1) {
        return 0;
    }
    if (x <= p) {
        return (m / pow(p, 2)) * (2 * p - 2 * x);
    } else {
        return (m / pow(1 - p, 2)) * (2 * p - 2 * x);
    }
}

double calculate_yt(double x, double t) {
    return (t / 0.2) * (0.2969 * sqrt(x) - 0.1260 * x - 0.3516 * pow(x, 2) + 0.2843 * pow(x, 3) - 0.1015 * pow(x, 4));
}

int main() {
    int m, p, xx;
    double hinge, def_angle_deg;

    cout << "Enter 4-digit NACA airfoil (m p xx): ";
    cin >> m >> p >> xx;

    cout << "Enter hinge position (0 to 1.0): ";
    cin >> hinge;
    hinge = std::clamp(hinge, 0.0, 1.0); 

    cout << "Enter angle of deflection (deg, + is down): ";
    cin >> def_angle_deg;

    double max_camber = m / 100.0;
    double max_camber_pos = p / 10.0;
    double max_thickness = xx / 100.0;

    double hinge_slope_rad = calculate_dyc_dx(hinge, max_camber, max_camber_pos);
    double hinge_slope_deg = atan(hinge_slope_rad) * 180 / M_PI;

    double total_deflection_deg = def_angle_deg + hinge_slope_deg;
    double total_deflection_rad = total_deflection_deg * M_PI / 180.0;

    int num_points = 100;

    cout << fixed << setprecision(6);

    vector<pair<double, double>> upper_surface;
    vector<pair<double, double>> lower_surface;

    for (int i = 0; i <= num_points; ++i) {
        double x = (1 - cos(M_PI * i / num_points)) / 2;
        double yc = calculate_yc(x, max_camber, max_camber_pos);
        double yt = calculate_yt(x, max_thickness);
        double dyc_dx = calculate_dyc_dx(x, max_camber, max_camber_pos);
        double theta = atan(dyc_dx);

        double xu = x - yt * sin(theta);
        double yu = yc + yt * cos(theta);
        double xl = x + yt * sin(theta);
        double yl = yc - yt * cos(theta);

        if (x < hinge && def_angle_deg != 0) {
            double hinge_yc = calculate_yc(hinge, max_camber, max_camber_pos);

            double alpha = total_deflection_rad * (1 - x / hinge);
            double cos_alpha = cos(alpha);
            double sin_alpha = sin(alpha);

            double xu_original = xu;
            double yu_original = yu;
            double xl_original = xl;
            double yl_original = yl;

            xu = hinge + (xu_original - hinge) * cos_alpha - (yu_original - hinge_yc) * sin_alpha;
            yu = hinge_yc + (xu_original - hinge) * sin_alpha + (yu_original - hinge_yc) * cos_alpha;

            xl = hinge + (xl_original - hinge) * cos_alpha - (yl_original - hinge_yc) * sin_alpha;
            yl = hinge_yc + (xl_original - hinge) * sin_alpha + (yl_original - hinge_yc) * cos_alpha;
        }

        upper_surface.push_back({xu, yu});
        lower_surface.push_back({xl, yl});
    }

    cout << "NACA" << m << p << xx << ", hinge = " << hinge << ", def angle = " << def_angle_deg << endl;

    for (int i = upper_surface.size() - 1; i >= 0; i--)
        cout << upper_surface[i].first << " " << upper_surface[i].second << endl;

    for (size_t i = 1; i < lower_surface.size(); ++i)
        cout << lower_surface[i].first << " " << lower_surface[i].second << endl;

    return 0;
}