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

	this->dimensions = QRectF(-6000, -3000, 12000, 6000);
	viewer = new AbstractGraphicViewer(this->frame, this->dimensions);
	auto [r, e] = viewer->add_robot(15, 15, 0, 100, QColor("Blue"));
	robot_polygon = r;   // declarar en .h


	this->resize(900,450);
	viewer->show();
	const auto rob = viewer->add_robot(ROBOT_LENGTH, ROBOT_LENGTH, 0, 190, QColor("Blue"));
	robot_polygon = std::get<0>(rob);

	viewer_room = new AbstractGraphicViewer(this->frame_room, this->dimensions);
	auto [rr, re] = viewer_room->add_robot(ROBOT_LENGTH, ROBOT_LENGTH, 0, 190, QColor("Blue"));
	robot_room_draw = rr;

	// draw room in viewer_room
	viewer_room->scene.addRect(this->dimensions, QPen(Qt::black, 30));
	viewer_room->show();

	// initialise robot pose
	robot_pose.setIdentity();
	robot_pose.translate(Eigen::Vector2d(0.0,0.0));


	connect(viewer, &AbstractGraphicViewer::new_mouse_coordinates, this, &SpecificWorker::new_target_slot);

}



void SpecificWorker::compute()
{
	// Robot perception
	RoboCompLidar3D::TPoints filter_data;
	RoboCompLidar3D::TData data;
	try
	{
		// filtrar lidar
		// detectar minimo lidar filtrado
		// decidimos si return o parar y girar en base a la lectura reciente del lidar

		data = lidar3d_proxy->getLidarDataWithThreshold2d("pearl", 15000, 1);
		draw_lidar(data.points, &viewer->scene);
		if (data.points.empty()){qWarning()<<"No points received"; return;}

		// opcional para quitar puntos liDAR aislados del resto
		data.points = filter_isolated_points(data.points , 200);

		//qInfo() << data.points.size();

		// auto filter_data_ = filter_min_distance_cppitertools(data.points);
		// if (filter_data_.has_value())
		// {
		// 	filter_data = filter_data_.value();
		//
		// } else
		// 	return;
	}
    catch (const Ice::Exception& e){ std::cout << e << " " << "Conexion con laser"<< std::endl; }

	auto corners = room_detector.compute_corners(data.points, &viewer->scene); // Detecta las esquinas de la habitacion

    auto match = hungarian.match(corners, room.transform_corners_to(robot_pose.inverse())); // Junta las esquinas detectadas con las esquinas de la habitacion
	for (auto &m:match)
	{
		// Imprimir los puntos del match
		qDebug () << std::get<0>(std::get<0>(m)).x() << " " << std::get<0>(std::get<0>(m)).y();
		qDebug () << std::get<0>(std::get<1>(m)).x() << " " << std::get<0>(std::get<1>(m)).y();
	}

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
		return;

	robot_pose.translate(Eigen::Vector2d(r(0), r(1)));
	robot_pose.rotate(r[2]);

	robot_room_draw->setPos(robot_pose.translation().x(), robot_pose.translation().y());
	double angle = std::atan2(robot_pose.rotation()(1, 0), robot_pose.rotation()(0, 0));
	robot_room_draw->setRotation(angle);


	//StateMachine(data);
}

void SpecificWorker::StateMachine(const RoboCompLidar3D::TData &data)
{
	float adv = 0.f, rot = 0.f;

	std::tuple<State, float, float> result;	//State -> enum class
	switch(state)
	{
	case State:: IDLE:
		break;
	case State::FORWARD:
		decision = false;
		result = FORWARD_method(data.points);
		break;
	case State:: TURN:
		result = TURN_method(data.points);
		break;
	case State:: FOLLOW_WALL:
		decision = false;
		result = FOLLOW_WALL_method(data.points);
		break;
	case State:: SPIRAL:
		decision = false;
		result = SPIRAL_method(data.points);
		break;
	}
	state = std::get<State>(result);
	adv = std::get<1>(result);
	rot = std::get<2>(result);
	//try-catch block to send velocities to the robot
	try
	{
		omnirobot_proxy->setSpeedBase(0, adv, rot);
	}
	catch (const Ice::Exception& e){ std::cout << e << " " << "Establecer velocidad"<< std::endl; }
}

std::tuple<SpecificWorker::State, float, float> SpecificWorker::FORWARD_method(const RoboCompLidar3D::TPoints  &ldata)
{
	int offset = ldata.size()/2 - 15;
	auto min = std::min_element(ldata.begin()+offset, ldata.end()-offset, [](const auto &p1, const auto &p2)
			{return p1.r < p2.r;});
	//qInfo() << min->phi;
	qInfo() << "Estado Forward:" << min->r;

	//Condicion de salida
	if (min->r < 670.f)
	{
		state = State::TURN;
		return std::make_tuple(state, 0.f, 0.f);
	}
	// lo mio
	return std::make_tuple(State::FORWARD, 1000.f, 0.f);
	
}

std::tuple<SpecificWorker::State, float, float> SpecificWorker::TURN_method(const RoboCompLidar3D::TPoints  &ldata)
{
	static std::tuple<SpecificWorker::State, float, float> result;
	int offset = ldata.size()/2 - 15;
	auto min = std::min_element(ldata.begin()+offset, ldata.end()-offset, [](const auto &p1, const auto &p2)
			{return p1.r < p2.r;});

	auto min_angulo = std::min_element(ldata.begin()+offset, ldata.end()-offset, [](const auto &p1, const auto &p2)
		{return abs(p1.phi) < abs(p2.phi);});


	qInfo() << "Estado Turn:" << min->r;


	//Condicion de salida
	if (min->r >= 670.f)
	{
		std::random_device rd;
		std::mt19937 gen(rd());
		std::uniform_int_distribution<> distrib(1, 10);
		int random = distrib(gen);
		if (random > 5)
			state = State::FORWARD;
		else
			state = State::FOLLOW_WALL;

		return std::make_tuple(state, 0.f, 0.f);
	}
	// Si pared a la izquierda gira en sentido horario
	qInfo() << "	Angulo" << min->phi;

	if (!decision) //|| veces == max_veces
	{
		if (min_angulo->phi < 0)
		{
			decision = true;
			result = std::make_tuple(State::TURN, 0.f, 0.7f);
			return result;
		}
		else 	// Si pared a la derecha gira en sentido antihorario
		{
			decision = true;
			result = std::make_tuple(State::TURN, 0.f, -0.7f);
			return result;
		}
	}
	return result;
}

std::tuple<SpecificWorker::State, float, float> SpecificWorker::FOLLOW_WALL_method(const RoboCompLidar3D::TPoints  &ldata)
{
	int offset = ldata.size()/2 - 15;
	auto min = std::min_element(ldata.begin()+offset, ldata.end()-offset, [](const auto &p1, const auto &p2)
			{return p1.r < p2.r;});
	auto min_angulo = std::min_element(ldata.begin()+offset, ldata.end()-offset, [](const auto &p1, const auto &p2)
	{return abs(p1.phi) < abs(p2.phi);});
	qInfo() << "Follow wall Distancia:" << min->r;
	qInfo() << "	Follow wall Angulo:" << min_angulo->phi;

	//Condicion de salida
	if (min->r < 650.f)
	{
		return std::make_tuple(State::TURN, 0.f, 0.f);
	}

	if (min->r > 670.f)
	{
		if (min_angulo->phi < 0)
		{
			return std::make_tuple(State::FOLLOW_WALL, 500.f, -0.5f);
		}
		else
		{
			return std::make_tuple(State::FOLLOW_WALL, 500.f, 0.5f);
		}
	}
	else
	{
		return std::make_tuple(State::FOLLOW_WALL, 1000.f, 0.f);
	}

}

std::tuple<SpecificWorker::State, float, float> SpecificWorker::SPIRAL_method(const RoboCompLidar3D::TPoints  &ldata)
{
	int offset = ldata.size()/2 - 15;
	auto min = std::min_element(ldata.begin()+offset, ldata.end()-offset, [](const auto &p1, const auto &p2)
			{return p1.r < p2.r;});
	//qInfo() << min->phi;
	//Condicion de salida
	if (min->r < 800.f)
	{
		state = State::TURN;
		return std::make_tuple(state, 0.f, 0.f);
	}
	float rot = 1.f;
	float adv = 5.f;
	static float dism = 0.001;
	dism += 0.001;
	static float aum = 5;
	aum += 5;
	return std::make_tuple(State::SPIRAL, adv+aum, rot-dism);

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

void SpecificWorker::draw_lidar(const auto &points, QGraphicsScene* scene)
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
	for (const auto &p : points)
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

void SpecificWorker::new_target_slot(QPointF)
{
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

