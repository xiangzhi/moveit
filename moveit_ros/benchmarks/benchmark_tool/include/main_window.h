/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Mario Prats
 */

#ifndef BT_MAIN_WINDOW_
#define BT_MAIN_WINDOW_

#include <frame_marker.h>
#include <trajectory.h>
#include <job_processing.h>

#include <QtGui/QMainWindow>
#include <QTimer>
#include "ui_main_window.h"

#include <rviz/render_panel.h>
#include <rviz/visualization_manager.h>
#include <rviz/robot/robot.h>

#include <moveit/render_tools/kinematic_state_visualization.h>
#include <moveit/planning_scene_rviz_plugin/planning_scene_display.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/warehouse/planning_scene_storage.h>
#include <moveit/warehouse/constraints_storage.h>
#include <moveit/warehouse/state_storage.h>
#include <moveit/robot_interaction/robot_interaction.h>

namespace benchmark_tool
{

class MainWindow : public QMainWindow
{
Q_OBJECT

public:
  MainWindow(int argc, char **argv, QWidget *parent = 0);

  ~MainWindow();
public Q_SLOTS:

  void exitActionTriggered(bool);
  void openActionTriggered(bool);
  void planningGroupChanged(const QString &text);
  void dbConnectButtonClicked();
  void dbConnectButtonClickedBackgroundJob();
  void robotInteractionButtonClicked();

  void loadSceneButtonClicked(void);
  void loadSceneButtonClicked(QListWidgetItem *item);

  //Goals and states
  void goalPoseFeedback(visualization_msgs::InteractiveMarkerFeedback &feedback);
  void createGoalPoseButtonClicked(void);
  void removeSelectedGoalsButtonClicked(void);
  void removeAllGoalsButtonClicked(void);
  void goalPoseSelectionChanged(void);
  void switchGoalVisibilityButtonClicked(void);
  void goalPoseDoubleClicked(QListWidgetItem *item);
  void copySelectedGoalPoses(void);
  void visibleAxisChanged(int state);
  void checkGoalsInCollision(void);
  void checkGoalsReachable(void);
  void loadBenchmarkResults(void);
  void updateMarkerStateFromName(const std::string &name, const GripperMarker::GripperMarkerState &state);

  void saveStartStateButtonClicked(void);
  void removeSelectedStatesButtonClicked(void);
  void removeAllStatesButtonClicked(void);
  void startStateItemDoubleClicked(QListWidgetItem * item);

  void loadGoalsFromDBButtonClicked(void);
  void saveGoalsOnDBButtonClicked(void);
  void deleteGoalsOnDBButtonClicked(void);
  void loadStatesFromDBButtonClicked(void);
  void saveStatesOnDBButtonClicked(void);
  void deleteStatesOnDBButtonClicked(void);

  //Trajectories
  void trajectorySelectionChanged(void);
  void createTrajectoryButtonClicked(void);

  //main loop processing
  void MainLoop();


private:
  const static char * ROBOT_DESCRIPTION_PARAM;
  const static char * ROBOT_DESCRIPTION_SEMANTIC_PARAM;
  const static unsigned int DEFAULT_WAREHOUSE_PORT;

  Ui::MainWindow ui_;

  //rviz
  rviz::RenderPanel *render_panel_;
  rviz::VisualizationManager *visualization_manager_;
  moveit_rviz_plugin::PlanningSceneDisplay *scene_display_;

  void scheduleStateUpdate();
  void scheduleStateUpdateBackgroundJob();
  bool isIKSolutionCollisionFree(kinematic_state::JointStateGroup *group, const std::vector<double> &ik_solution);
  bool configure();
  void loadNewRobot(const std::string &urdf_path, const std::string &srdf_path);
  void setItemSelectionInList(const std::string &item_name, bool selection, QListWidget *list);
  void selectItemJob(QListWidgetItem *item, bool flag);

  //robot interaction
  robot_interaction::RobotInteractionPtr robot_interaction_;
  rviz::Display *int_marker_display_;

  //Warehouse
  boost::shared_ptr<moveit_warehouse::PlanningSceneStorage> planning_scene_storage_;
  boost::shared_ptr<moveit_warehouse::ConstraintsStorage> constraints_storage_;
  boost::shared_ptr<moveit_warehouse::RobotStateStorage> robot_state_storage_;

  void populatePlanningSceneList(void);

  //Goals and start states
  robot_interaction::RobotInteraction::InteractionHandlerPtr query_goal_state_;

  EigenSTL::map_string_Affine3d goals_initial_pose_;
  bool goal_pose_dragging_;

  typedef std::map<std::string, GripperMarkerPtr> GoalPoseMap;
  typedef std::pair<std::string, GripperMarkerPtr> GoalPosePair;
  GoalPoseMap goal_poses_;

  class StartState
  {
  public:
    moveit_msgs::RobotState state_msg;
    bool selected;

    StartState(): selected(false) {}
    StartState(const moveit_msgs::RobotState &state): state_msg(state), selected(false) {}
    StartState(const moveit_msgs::RobotState &state, bool is_selected): state_msg(state), selected(is_selected) {}
  };
  typedef boost::shared_ptr<StartState> StartStatePtr;

  typedef std::map<std::string, StartStatePtr> StartStateMap;
  typedef std::pair<std::string, StartStatePtr> StartStatePair;
  StartStateMap start_states_;

  void populateGoalPosesList();
  void populateStartStatesList();
  void populateTrajectoriesList();
  void computeGoalPoseDoubleClicked(QListWidgetItem * item);
  void switchGoalPoseMarkerSelection(const std::string &marker_name);
  typedef std::pair<visualization_msgs::InteractiveMarker, boost::shared_ptr<rviz::InteractiveMarker> > MsgMarkerPair;

  void checkIfGoalInCollision(const std::string & goal_name);
  void checkIfGoalReachable(const std::string &goal_name, bool update_if_reachable = false);
  void computeLoadBenchmarkResults(const std::string &file);

  void updateGoalPoseMarkers(float wall_dt, float ros_dt);

  //Trajectories
  void switchTrajectorySelection(const std::string &marker_name);

  typedef std::map<std::string, TrajectoryPtr> TrajectoryMap;
  typedef std::pair<std::string, TrajectoryPtr> TrajectoryPair;
  TrajectoryMap trajectories_;

  void createTrajectoryStartMarker(const GripperMarker &marker);

  //Background processing
  void loadSceneButtonClickedBackgroundJob(void);

  //Foreground processing
  const static unsigned int MAIN_LOOP_RATE = 20; //calls to executeMainLoopJobs per second
  boost::shared_ptr<QTimer> main_loop_jobs_timer_;

  //Status and logging
  typedef enum {STATUS_WARN, STATUS_ERROR, STATUS_INFO} StatusType;
  void setStatus(StatusType st, const QString &text)
  {
    if (st == STATUS_WARN)
    {
      ROS_WARN_STREAM(text.toStdString());
      ui_.status_label->setText(text);
    }
    else if (st == STATUS_ERROR)
    {
      ROS_ERROR_STREAM(text.toStdString());
      ui_.status_label->setText(text);
    }
    else if (st == STATUS_INFO)
    {
      ui_.status_label->setText(text);
    }
  }

  void setStatusFromBackground(StatusType st, const QString &text)
  {
    JobProcessing::addMainLoopJob(boost::bind(&MainWindow::setStatus, this, st, text));
  }

};

}

#endif
