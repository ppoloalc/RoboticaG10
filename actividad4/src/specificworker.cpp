/*
 *    Copyright (C) 2025 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"

#include <ranges>
#include <cppitertools/groupby.hpp>
#include<cppitertools/enumerate.hpp>
#ifdef emit
#undef emit
#endif
#include <algorithm>
#include <execution>
#include <expected>
#include <chrono>
#include <random>




SpecificWorker::SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check) : GenericWorker(configLoader, tprx)
{
	this->startup_check_flag = startup_check;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		#ifdef HIBERNATION_ENABLED
			hibernationChecker.start(500);
		#endif
		
		// Example statemachine:
		/***
		//Your definition for the statesmachine (if you dont want use a execute function, use nullptr)
		states["CustomState"] = std::make_unique<GRAFCETStep>("CustomState", period, 
															std::bind(&SpecificWorker::customLoop, this),  // Cyclic function
															std::bind(&SpecificWorker::customEnter, this), // On-enter function
															std::bind(&SpecificWorker::customExit, this)); // On-exit function

		//Add your definition of transitions (addTransition(originOfSignal, signal, dstState))
		states["CustomState"]->addTransition(states["CustomState"].get(), SIGNAL(entered()), states["OtherState"].get());
		states["Compute"]->addTransition(this, SIGNAL(customSignal()), states["CustomState"].get()); //Define your signal in the .h file under the "Signals" section.

		//Add your custom state
		statemachine.addState(states["CustomState"].get());
		***/

		statemachine.setChildMode(QState::ExclusiveStates);
		statemachine.start();

		auto error = statemachine.errorString();
		if (error.length() > 0){
			qWarning() << error;
			throw error;
		}
	}
}

SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}


void SpecificWorker::initialize()
{
    std::cout << "initialize worker" << std::endl;

    //initializeCODE

    /////////GET PARAMS, OPEND DEVICES....////////
    //int period = configLoader.get<int>("Period.Compute") //NOTE: If you want get period of compute use getPeriod("compute")
    //std::string device = configLoader.get<std::string>("Device.name")

	if(this->startup_check_flag)
	{
		this->startup_check();
	}

	else
	{
		// Viewer
		viewer = new AbstractGraphicViewer(this->frame, params.GRID_MAX_DIM);
		auto [r, e] = viewer->add_robot(params.ROBOT_WIDTH, params.ROBOT_LENGTH, 0, 100, QColor("Blue"));
		robot_draw = r;
		//viewer->show();


		viewer_room = new AbstractGraphicViewer(this->frame_room, params.GRID_MAX_DIM);
		auto [rr, re] = viewer_room->add_robot(params.ROBOT_WIDTH, params.ROBOT_LENGTH, 0, 100, QColor("Blue"));
		robot_room_draw = rr;
		// draw room in viewer_room


		hab = viewer_room->scene.addRect(nominal_rooms[room_index].rect(), QPen(Qt::black, 30));



		//viewer_room->show();
		show();


		// initialise robot pose
		robot_pose.setIdentity();
		robot_pose.translate(Eigen::Vector2d(0.0,0.0));


		// time series plotter for match error
		TimeSeriesPlotter::Config plotConfig;
		plotConfig.title = "Maximum Match Error Over Time";
		plotConfig.yAxisLabel = "Error (mm)";
		plotConfig.timeWindowSeconds = 15.0; // Show a 15-second window
		plotConfig.autoScaleY = false;       // We will set a fixed range
		plotConfig.yMin = 0;
		plotConfig.yMax = 1000;
		time_series_plotter = std::make_unique<TimeSeriesPlotter>(frame_plot_error, plotConfig);
		match_error_graph = time_series_plotter->addGraph("", Qt::blue);


		// stop robot
		move_robot(0, 0, 0);
	}
}



void SpecificWorker::compute()
{
	RoboCompLidar3D::TPoints data = read_data();
	doors = door_detector.detect(data, &viewer->scene);
   data= door_detector.filter_points(data, &viewer->scene);


   // compute corners
   const auto &[corners, lines] = room_detector.compute_corners(data, &viewer->scene);
   const auto center_opt = room_detector.estimate_center_from_walls(lines);
   draw_lidar(data, *center_opt, &viewer->scene);
	draw_doors(nominal_rooms[room_index].doors);

   // match corners  transforming first nominal corners to robot's frame
	Match match;

   	 match = hungarian.match(corners,
											 nominal_rooms[room_index].transform_corners_to(robot_pose.inverse()));




   // compute max of  match error
   float max_match_error = 99999.f;
   if (not match.empty())
   {
       const auto max_error_iter = std::ranges::max_element(match, [](const auto &a, const auto &b)
           { return std::get<2>(a) < std::get<2>(b); });
       max_match_error = static_cast<float>(std::get<2>(*max_error_iter));
       time_series_plotter->addDataPoint(0, max_match_error);
       //print_match(match, max_match_error); //debugging
   }

	//qInfo() << max_match_error;



   // update robot pose
    if (localised)
        update_robot_pose(corners, match);


   // Process state machine
   RetVal ret_val = process_state(data, corners, match, viewer);
   auto [st, adv, rot] = ret_val;
   state = st;


   // Send movements commands to the robot constrained by the match_error
   qInfo() << __FUNCTION__ << "Adv: " << adv << " Rot: " << rot;
   move_robot(adv, rot, max_match_error);


   // draw robot in viewer
   robot_room_draw->setPos(robot_pose.translation().x(), robot_pose.translation().y());
   const double angle = qRadiansToDegrees(std::atan2(robot_pose.rotation()(1, 0), robot_pose.rotation()(0, 0)));
   robot_room_draw->setRotation(angle);


   // update GUI
   time_series_plotter->update();
   /*lcdNumber_adv->display(adv);
   lcdNumber_rot->display(rot);*/
   lcdNumber_x->display(robot_pose.translation().x());
   lcdNumber_y->display(robot_pose.translation().y());
   lcdNumber_angle->display(angle);
   last_time = std::chrono::high_resolution_clock::now();
	label_state->setText(to_string(state));
}

std::optional<RoboCompLidar3D::TPoints> SpecificWorker::filter_min_distance_cppitertools(const RoboCompLidar3D::TPoints& points)
{
	if (points.empty()){return{};}
	RoboCompLidar3D::TPoints result;
	result.reserve(points.size());
	for (auto&& [angle, group]:iter::groupby(points, [](const auto &p)
		{float multiplier = std::pow(10.0f, 2); return std::floor(p.phi*multiplier/multiplier); }))
	{
		auto min = std::min_element(group.begin(), group.end(), [](const auto &p1, const auto &p2)
			{return p1.r < p2.r;});
		//if (min->r>	450 && min->phi > -std::numbers::pi/2 && min->phi < std::numbers::pi/2 )
		result.emplace_back(*min);
	}
	return result;
}

SpecificWorker::RetVal SpecificWorker::goto_door(const RoboCompLidar3D::TPoints& points)
{

	const auto centro = robot_pose.inverse() * nominal_rooms[room_index].doors[current_door].center_before(robot_pose.translation(), 1000.f).cast<double>(); //robot_pose debe ser Vector2d
	// exit condition -> Llega al centro antes de la puert
	if (centro.norm() < 500.f)
	{
		return {STATE::ORIENT_TO_DOOR, 0.f, 0.f};
	}

	const auto &[adv, rot] = robot_controller(centro.cast<float>());
	static QGraphicsItem* item = nullptr;
	if (item != nullptr)
	{
		viewer->scene.removeItem(item);
		delete item;
	}

	item=viewer->scene.addEllipse(centro.x() - 100, centro.y() - 100, 200, 200, QPen(Qt::magenta, 30));
	return {STATE::GOTO_DOOR, adv*0.9, rot};
}

SpecificWorker::RetVal SpecificWorker::orient_to_door(const RoboCompLidar3D::TPoints& points)
{
	const auto centro_puerta = robot_pose.inverse() * ((nominal_rooms[room_index].doors[current_door].global_p1+nominal_rooms[room_index].doors[current_door].global_p2)/2).cast<double>();

	qInfo() << "Centro puerta" << centro_puerta.x() << centro_puerta.y();


	//Exit condition si mirando a puerta
	//if (centro_puerta.norm() < 500.f)
	if (centro_puerta.norm() < 500.f)
	{
		return {STATE::CROSS_DOOR, 0.f, 0.f};
	}
	const auto &[adv, rot] = robot_controller(centro_puerta.cast<float>());
	return {STATE::ORIENT_TO_DOOR, 100.f, rot};
}

SpecificWorker::RetVal SpecificWorker::cross_door(const RoboCompLidar3D::TPoints& points)
{
	static int cont = 0;

	if (cont == 30)
	{
		room_index = (room_index + 1) % 2;
		if (room_index % 2 == 0){
			rojo = true;
		}
		else
		{
			rojo = false;
		}
		localised = false;
		cont = 0;
		viewer_room->scene.removeItem(hab);

		hab = viewer_room->scene.addRect(nominal_rooms[room_index].rect(), QPen(Qt::black, 30));


		return {STATE::GOTO_ROOM_CENTER, 0.f, 0.f};
	}
	cont ++;
	return {STATE::CROSS_DOOR, 500.f, 0.f};
}

SpecificWorker::RetVal SpecificWorker::localise(const Match& match)
{

}

SpecificWorker::RetVal SpecificWorker::goto_room_center(const RoboCompLidar3D::TPoints& points, AbstractGraphicViewer* viewer)
{
	auto center = room_detector.estimate_center_from_walls();
	if (not center.has_value())return{};

	//exit
	if (center.value().norm() < 300.f)
	{
		return {STATE::TURN, 0.f, 0.f};
	}

	Eigen::Vector2f center_point = center.value().cast<float>();
	auto [adv, rotVel] = robot_controller(center_point);
	static QGraphicsItem* item = nullptr;
	if (item != nullptr)
	{
		viewer->scene.removeItem(item);
		delete item;
	}

	item=viewer->scene.addEllipse(center.value().x() - 100, center.value().y() - 100, 200, 200, QPen(Qt::magenta, 30));

	return {STATE::GOTO_ROOM_CENTER, adv, rotVel};
}

SpecificWorker::RetVal SpecificWorker::turn(const Corners& corners, const RoboCompLidar3D::TPoints& points, const Match& match)
{

	//const auto success = false;
	//const auto spin = 0;
	std::tuple<bool, int> cuadro;
	if (rojo)
	{
		cuadro = image_processor.check_colour_patch_in_image(camera360rgb_proxy, "red", label_img);
	}
	else
	{
		 cuadro = image_processor.check_colour_patch_in_image(camera360rgb_proxy, "green", label_img);
	}

	// exit condition
	if (std::get<0>(cuadro))
	{

		// call localiser()
		localised = true; //Encuentra cuadrado rojo
		bool error = update_robot_pose(corners, match);
		qInfo() << "Error localizacion" << error;

		for(auto &door : doors)
		{
			door.global_p1 = nominal_rooms[room_index].get_projection_of_point_on_closest_wall((robot_pose * door.p1.cast<double>()).cast<float>());
			door.global_p2 = nominal_rooms[room_index].get_projection_of_point_on_closest_wall((robot_pose * door.p2.cast<double>()).cast<float>());
		}

		nominal_rooms[room_index].doors = doors;
		current_door = 0;

		draw_doors(doors);

		return {STATE::GOTO_DOOR, 0.f, 0.f};
	}

	return {STATE::TURN, 0.f,  std::get<1>(cuadro) * 0.3};
}

SpecificWorker::RetVal SpecificWorker::process_state(const RoboCompLidar3D::TPoints& data, const Corners& corners,
                                                     const Match& match, AbstractGraphicViewer* viewer)
{
	RetVal val;
	switch (state)
	{
		case STATE::GOTO_ROOM_CENTER:
			val = goto_room_center(data, viewer);
			break;
		case STATE::TURN:
			val = turn(corners, data, match);
			break;
		case STATE::ORIENT_TO_DOOR:
			val = orient_to_door(data);
			break;
		case STATE::CROSS_DOOR:
			val = cross_door(data);
			break;
		case STATE::LOCALISE:
			break;
		case STATE::GOTO_DOOR:
			val = goto_door(data);
			break;
		case STATE::IDLE:
			break;

	}

	return val;
}

void SpecificWorker::draw_lidar(const RoboCompLidar3D::TPoints &filtered_points, Eigen::Vector2d center, QGraphicsScene *scene)
{
	static std::vector<QGraphicsItem*> draw_points;
	for (const auto &p : draw_points)
	{
		scene->removeItem(p);
		delete p;
	}
	draw_points.clear();

	const QColor color("LightGreen");
	const QPen pen(color, 10);
	//const QBrush brush(color, Qt::SolidPattern);
	for (const auto &p : filtered_points)
	{
		const auto dp = scene->addRect(-25, -25, 50, 50, pen);
		dp->setPos(p.x, p.y);
		draw_points.push_back(dp);   // add to the list of points to be deleted next time
	}
}


void SpecificWorker::emergency()
{
	std::cout << "Emergency worker" << std::endl;
	//emergencyCODE
	//
	//if (SUCCESSFUL) //The componet is safe for continue
	//  emmit goToRestore()
}

RoboCompLidar3D::TPoints SpecificWorker::read_data()
{
	// Robot perception
	RoboCompLidar3D::TData data;
	try
	{
		data = lidar3d_proxy->getLidarDataWithThreshold2d("helios", 15000, 1);
	}
	catch (const Ice::Exception& e){ std::cout << e << " " << "Conexion con laser"<< std::endl; return{};}
	if (data.points.empty()){qWarning()<<"No points received"; return{};}
	// opcional para quitar puntos liDAR aislados del resto
	data.points = filter_isolated_points(data.points , 100);

	return data.points;
}

RoboCompLidar3D::TPoints SpecificWorker::filter_isolated_points(const RoboCompLidar3D::TPoints &points, float d) // set to 200mm
{
	if (points.empty()) return {};

	const float d_squared = d * d;  // Avoid sqrt by comparing squared distances
	std::vector<bool> hasNeighbor(points.size(), false);

	// Create index vector for parallel iteration
	std::vector<size_t> indices(points.size());
	std::iota(indices.begin(), indices.end(), size_t{0});

	// Parallelize outer loop - each thread checks one point
	std::for_each(std::execution::par, indices.begin(), indices.end(), [&](size_t i)
		{
			const auto& p1 = points[i];
			// Sequential inner loop (avoid nested parallelism)
			for (auto &&[j,p2] : iter::enumerate(points))
			{
				if (i == j) continue;
				const float dx = p1.x - p2.x;
				const float dy = p1.y - p2.y;
				if (dx * dx + dy * dy <= d_squared)
				{
					hasNeighbor[i] = true;
					break;
				}
			}
	});

	// Collect results
	std::vector<RoboCompLidar3D::TPoint> result;
	result.reserve(points.size());
	for (auto &&[i, p] : iter::enumerate(points))
		if (hasNeighbor[i])
			result.push_back(points[i]);
	return result;
}

void SpecificWorker::draw_doors(const Doors& doors)
{
	static std::vector<QGraphicsItem*> draw_points;
	for (const auto &d : draw_points)
	{
		viewer_room->scene.removeItem(d);
		delete d;
	}
	draw_points.clear();

	for (const auto &door : doors)
	{
		auto item = viewer_room->scene.addEllipse(-100, -100, 200, 200, QPen(QColor("green")), QBrush(QColor("green")));
		item->setPos(door.global_p1.x(), door.global_p1.y());
		draw_points.push_back(item);
		item = viewer_room->scene.addEllipse(-100, -100, 200, 200, QPen(QColor("green")), QBrush(QColor("green")));
		item->setPos(door.global_p2.x(), door.global_p2.y());
		draw_points.push_back(item);
		const auto item2 = viewer_room->scene.addLine(door.global_p1.x(), door.global_p1.y(), door.global_p2.x(), door.global_p2.y(), QPen(QColor("magenta"), 30));
		draw_points.push_back(item2);
	}
}

bool SpecificWorker::update_robot_pose(const Corners& corners, const Match& match)
{
	Eigen::MatrixXd W(corners.size() * 2, 3);
	Eigen::VectorXd b(corners.size() * 2);
	for (auto &&[i, m]: match | iter::enumerate)
	{
		auto &[meas_c, nom_c, _] = m;
		auto &[p_meas, __, ___] = meas_c;
		auto &[p_nom, ____, _____] = nom_c;
		b(2 * i)     = p_nom.x() - p_meas.x();
		b(2 * i + 1) = p_nom.y() - p_meas.y();
		W.block<1, 3>(2 * i, 0)     << 1.0, 0.0, -p_meas.y();
		W.block<1, 3>(2 * i + 1, 0) << 0.0, 1.0, p_meas.x();
	}
	// estimate new pose with pseudoinverse
	const Eigen::Vector3d r = (W.transpose() * W).inverse() * W.transpose() * b;
	std::cout << r << std::endl;
	qInfo() << "--------------------";


	if (r.array().isNaN().any())
		return false;

	robot_pose.translate(Eigen::Vector2d(r(0), r(1)));
	robot_pose.rotate(r[2]);

	robot_room_draw->setPos(robot_pose.translation().x(), robot_pose.translation().y());
	double angle = std::atan2(robot_pose.rotation()(1, 0), robot_pose.rotation()(0, 0));
	robot_room_draw->setRotation(angle);
	return true;
}

void SpecificWorker::move_robot(float adv, float rot, float max_match_error)
{
	try
	{
		omnirobot_proxy->setSpeedBase(0, adv, rot);
	}
	catch (const Ice::Exception& e){ std::cout << e << " " << "Establecer velocidad"<< std::endl; }

}

std::tuple<float, float> SpecificWorker::robot_controller(const Eigen::Vector2f& target)
{
	float dist = target.norm();
	// exit
	//if (dist < 100.f) return {0.f, 0.f};

	// Do my thing
	float k = 0.5;
	// Angulo theta sub e (O grande sub e)
	float angle = atan2(target.x(), target.y());
	float rotVel = k * angle;

	// σ
	float sigma = M_PI / 6.0;   // 45 grados
	float adv = params.MAX_ADV_SPEED * exp((- angle * angle) / (2 * sigma * sigma));

	return {adv, rotVel};
}


//Execute one when exiting to emergencyState
void SpecificWorker::restore()
{
	std::cout << "Restore worker" << std::endl;
	//restoreCODE
	//Restore emergency component

}


int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, QCoreApplication::instance(), SLOT(quit()));
	return 0;
}



/**************************************/
// From the RoboCompDifferentialRobot you can call this methods:
// RoboCompDifferentialRobot::void this->differentialrobot_proxy->correctOdometer(int x, int z, float alpha)
// RoboCompDifferentialRobot::void this->differentialrobot_proxy->getBasePose(int x, int z, float alpha)
// RoboCompDifferentialRobot::void this->differentialrobot_proxy->getBaseState(RoboCompGenericBase::TBaseState state)
// RoboCompDifferentialRobot::void this->differentialrobot_proxy->resetOdometer()
// RoboCompDifferentialRobot::void this->differentialrobot_proxy->setOdometer(RoboCompGenericBase::TBaseState state)
// RoboCompDifferentialRobot::void this->differentialrobot_proxy->setOdometerPose(int x, int z, float alpha)
// RoboCompDifferentialRobot::void this->differentialrobot_proxy->setSpeedBase(float adv, float rot)
// RoboCompDifferentialRobot::void this->differentialrobot_proxy->stopBase()

/**************************************/
// From the RoboCompDifferentialRobot you can use this types:
// RoboCompDifferentialRobot::TMechParams

/**************************************/
// From the RoboCompLaser you can call this methods:
// RoboCompLaser::TLaserData this->laser_proxy->getLaserAndBStateData(RoboCompGenericBase::TBaseState bState)
// RoboCompLaser::LaserConfData this->laser_proxy->getLaserConfData()
// RoboCompLaser::TLaserData this->laser_proxy->getLaserData()

/**************************************/
// From the RoboCompLaser you can use this types:
// RoboCompLaser::LaserConfData
// RoboCompLaser::TData

