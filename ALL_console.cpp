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
    double hinge_TE, def_angle_deg_TE, hinge_LE, def_angle_deg_LE;
    double trimmer;

    cout << "Enter 4-digit NACA airfoil (m p xx): ";
    cin >> m >> p >> xx;

    cout << "Enter TE hinge position (0 to 1.0): ";
    cin >> hinge_TE;
    hinge_TE = std::clamp(hinge_TE, 0.0, 1.0); 

    cout << "Enter TE angle of deflection (deg, + is up): ";
    cin >> def_angle_deg_TE;
    
    cout << "Enter trimmer position (hinge to 1.0): ";
    cin >> trimmer;

    cout << "Enter LE hinge position (0 to 1.0): ";
    cin >> hinge_LE;
    hinge_LE = std::clamp(hinge_LE, 0.0, 1.0); 

    cout << "Enter LE angle of deflection (deg, - is up): ";
    cin >> def_angle_deg_LE;

    double max_camber = m / 100.0;
    double max_camber_pos = p / 10.0;
    double max_thickness = xx / 100.0;

    double hinge_slope_rad_TE = calculate_dyc_dx(hinge_TE, max_camber, max_camber_pos);
    double hinge_slope_deg_TE = atan(hinge_slope_rad_TE) * 180 / M_PI;

    double total_deflection_deg_TE = def_angle_deg_TE + hinge_slope_deg_TE;
    double total_deflection_rad_TE = total_deflection_deg_TE * M_PI / 180.0;

    double hinge_slope_rad_LE = calculate_dyc_dx(hinge_LE, max_camber, max_camber_pos);
    double hinge_slope_deg_LE = atan(hinge_slope_rad_LE) * 180 / M_PI;

    double total_deflection_deg_LE = def_angle_deg_LE + hinge_slope_deg_LE;
    double total_deflection_rad_LE = total_deflection_deg_LE * M_PI / 180.0;

    int num_points = 100;

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

        if (x > hinge_TE && def_angle_deg_TE != 0) {
            double hinge_yc = calculate_yc(hinge_TE, max_camber, max_camber_pos);

            double alpha = total_deflection_rad_TE * (x - hinge_TE) / (1 - hinge_TE);
            double cos_alpha = cos(alpha);
            double sin_alpha = sin(alpha);

            double xu_original = xu;
            double yu_original = yu;
            double xl_original = xl;
            double yl_original = yl;

            xu = hinge_TE + (xu_original - hinge_TE) * cos_alpha - (yu_original - hinge_yc) * sin_alpha;
            yu = hinge_yc + (xu_original - hinge_TE) * sin_alpha + (yu_original - hinge_yc) * cos_alpha;

            xl = hinge_TE + (xl_original - hinge_TE) * cos_alpha - (yl_original - hinge_yc) * sin_alpha;
            yl = hinge_yc + (xl_original - hinge_TE) * sin_alpha + (yl_original - hinge_yc) * cos_alpha;
            
            if (x >= trimmer) {
                double trimmer_yc = calculate_yc(trimmer, max_camber, max_camber_pos);
                
                double alpha_hinge = total_deflection_rad_TE * (trimmer - hinge_TE) / (1 - hinge_TE);
                double cos_alpha_hinge = cos(alpha_hinge);
                double sin_alpha_hinge = sin(alpha_hinge);
                
                double trimmer_hinged = hinge_TE + (trimmer - hinge_TE) * cos_alpha_hinge - (trimmer_yc - hinge_yc) * sin_alpha_hinge;
                double trimmer_yc_hinged = hinge_yc + (trimmer - hinge_TE) * sin_alpha_hinge + (trimmer_yc - hinge_yc) * cos_alpha_hinge;
            
                double alpha_trimm = -1.0 * (total_deflection_rad_TE * (trimmer - hinge_TE) / (1 - hinge_TE)) * ((x - trimmer) / (1 - trimmer));
                double cos_alpha_trimm = cos(alpha_trimm);
                double sin_alpha_trimm = sin(alpha_trimm);
                
                double xu_hinged = xu;
                double yu_hinged = yu;
                double xl_hinged = xl;
                double yl_hinged = yl;
                
                xu = trimmer_hinged + (xu_hinged - trimmer_hinged) * cos_alpha_trimm - (yu_hinged - trimmer_yc_hinged) * sin_alpha_trimm;
                yu = trimmer_yc_hinged + (xu_hinged - trimmer_hinged) * sin_alpha_trimm + (yu_hinged - trimmer_yc_hinged) * cos_alpha_trimm;
                
                xl = trimmer_hinged + (xl_hinged - trimmer_hinged) * cos_alpha_trimm - (yl_hinged - trimmer_yc_hinged) * sin_alpha_trimm;
                yl = trimmer_yc_hinged + (xl_hinged - trimmer_hinged) * sin_alpha_trimm + (yl_hinged - trimmer_yc_hinged) * cos_alpha_trimm;
                
            }
            
        }

         if (x < hinge_LE && def_angle_deg_LE != 0) {
            double hinge_yc = calculate_yc(hinge_LE, max_camber, max_camber_pos);

            double alpha = total_deflection_rad_LE * (1 - x / hinge_LE);
            double cos_alpha = cos(alpha);
            double sin_alpha = sin(alpha);

            double xu_original = xu;
            double yu_original = yu;
            double xl_original = xl;
            double yl_original = yl;

            xu = hinge_LE + (xu_original - hinge_LE) * cos_alpha - (yu_original - hinge_yc) * sin_alpha;
            yu = hinge_yc + (xu_original - hinge_LE) * sin_alpha + (yu_original - hinge_yc) * cos_alpha;

            xl = hinge_LE + (xl_original - hinge_LE) * cos_alpha - (yl_original - hinge_yc) * sin_alpha;
            yl = hinge_yc + (xl_original - hinge_LE) * sin_alpha + (yl_original - hinge_yc) * cos_alpha;
        }

        upper_surface.push_back({xu, yu});
        lower_surface.push_back({xl, yl});
    }

    cout << fixed << setprecision(2);
    cout << "NACA" << m << p << xx << ", hinge TE = " << hinge_TE << ", def angle TE = " << def_angle_deg_TE << ", hinge LE = " <<  hinge_LE << ", def angle LE = " << def_angle_deg_LE << ", hinge trimm = " << trimmer << endl;
    cout << fixed << setprecision(6);

    for (int i = upper_surface.size() - 1; i >= 0; i--)
        cout << upper_surface[i].first << " " << upper_surface[i].second << endl;

    for (size_t i = 1; i < lower_surface.size(); ++i)
        cout << lower_surface[i].first << " " << lower_surface[i].second << endl;

    return 0;
}