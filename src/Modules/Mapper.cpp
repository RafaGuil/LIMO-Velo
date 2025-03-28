#ifndef __OBJECTS_H__
#define __OBJECTS_H__
#include "Headers/Common.hpp"
#include "Headers/Utils.hpp"
#include "Headers/Objects.hpp"
#include "Headers/Publishers.hpp"
#include "Headers/PointClouds.hpp"
#include "Headers/Accumulator.hpp"
#include "Headers/Compensator.hpp"
#include "Headers/Localizator.hpp"
#include "Headers/Mapper.hpp"
#endif

extern struct Params Config;

// class Mapper
    // public:
        Mapper::Mapper() {  // TODO: (const KDTREE_OPTIONS& options) {
            this->init_tree();
        }

        void Mapper::add(Points& points, double time, bool downsample) {
            if (points.empty()) return;

            // If map doesn't exist, build it.
            if (not this->exists()) this->build_tree(points);
            else this->add_points(points, downsample);

            buffer_mapped_points_.insert(buffer_mapped_points_.end(), points.begin(), points.end());

            this->last_map_time = time;
        }

        Points Mapper::get() {
            return buffer_mapped_points_;
        }

        void Mapper::pop(int n) {
            while (buffer_mapped_points_.size() > n) {
                buffer_mapped_points_.pop_front();
            }
        }


        // void Mapper::add(Points& points, double time, bool downsample) {
        //     if (points.empty()) return;

        //     // If map doesn't exist, build it.
        //     if (not this->exists()) this->build_tree(points);
        //     else this->add_points(points, downsample);

        //     this->last_map_time = time;
        // }

        // Points Mapper::get() {
        //     for (Point p : this->map->getRootNodePoints()) buffer_mapped_points_.push_back(p);
        //     return buffer_mapped_points_;
        // }

        // void Mapper::pop(int n) {
        //     std::sort(buffer_mapped_points_.begin(), buffer_mapped_points_.end(),
        //         [](const Point& a, const Point& b){
        //             return a.time < b.time;
        //         }
        //     );
        //     PointVector to_delete; 
        //     int rest_points = buffer_mapped_points_.size() - n;
        //     for (int i = 0; i < rest_points; ++i) to_delete.push_back(buffer_mapped_points_[i]);
        //     this->map->Delete_Points(to_delete);
        // }


        int Mapper::size() {
            return this->map->size(); 
        }

        bool Mapper::exists() {
            return this->exists_tree();
        }

        // La funcion es llamada en el IKFoM
        Matches Mapper::match(const State& X, const Points& points) {
            Matches matches; // Vector de matches
            if (not this->exists()) return matches; // Si no se inicializa el mapa, no se puede hacer nada
            matches.reserve(points.size()); // Se va reservando el espacio para los matches
            
            omp_set_num_threads(MP_PROC_NUM); // para paralelizar el bucle
            #pragma omp parallel for
            for (int pi = 0; pi < points.size(); ++pi) {
                Point p = points[pi];

                // Direct approach: we match the point with a plane on the map
                Match match = this->match_plane(X * X.I_Rt_L() * p); // Traslada el punto y le hace match con un plano
                if (match.is_chosen()) matches.push_back(match); // is_chosen cuando is_plane = true (mirar en Plane.cpp)
            }

            return matches;
        }

        bool Mapper::hasToMap(double t) {
            if (this->last_map_time < 0) this->last_map_time = t;
            return t - this->last_map_time >= Config.full_rotation_time;
        }

    // private:
        void Mapper::init_tree() {  // TODO: (const KDTREE_OPTIONS& options) {
            this->map = KD_TREE<Point>::Ptr (new KD_TREE<Point>(Config.delete_KDTREE_points_param, Config.balance_param, Config.box_length_param));
        }

        void Mapper::build_tree(Points& points) {
            PointVector as_vector; for (Point p : points) as_vector.push_back(p);
            this->map->Build(as_vector);
        }

        void Mapper::add_points(Points& points, bool downsample) {
            PointVector as_vector; for (Point p : points) as_vector.push_back(p);
            this->map->Add_Points(as_vector, downsample);
        }

        bool Mapper::exists_tree() {
            return this->map->size() > 0;
        }

        Match Mapper::match_plane(const Point& p) {
            // 1) Declara un contenedor donde se guardarán los puntos vecinos
            PointVector near_points;
        
            // 2) Crea un vector para almacenar las distancias cuadráticas que tiene un tamaño = NUM_MATCH_POINTS
            vector<float> pointSearchSqDis(Config.NUM_MATCH_POINTS);
        
            // 3) Busca los k puntos más cercanos al punto p en el mapa, y almacena sus coordenadas y distancias
            this->map->Nearest_Search(p, Config.NUM_MATCH_POINTS, near_points, pointSearchSqDis);
        
            // 4) Construye un objeto Plane con esos puntos vecinos y sus distancias, y crea un Match con p y ese plano
            return Match(p, Plane(near_points, pointSearchSqDis));
        }