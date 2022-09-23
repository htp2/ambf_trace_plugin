//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2019-2021, AMBF
    (https://github.com/WPI-AIM/ambf)

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

    * Neither the name of authors nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

    \author    <henry.phalen@jhu.edu>
    \author    Henry Phalen
*/
//==============================================================================

#include <afFramework.h>
#include <boost/program_options.hpp>
#include <fstream>

#ifdef BUILD_WITH_ROS
#include <std_msgs/Bool.h>
#include <ros/ros.h>
#include <ambf_server/RosComBase.h>
#endif // BUILD_WITH_ROS

using namespace ambf;

// TODO: allow passing in a list of bodies, for multiple traces
// TODO: allow specification of color, line_width in args

class afTracePlugin : public afSimulatorPlugin
{
    int init(int argc, char **argv, const afWorldPtr a_afWorld)
    {
        namespace p_opt = boost::program_options;
        p_opt::options_description cmd_opts("ambf_trace_plugin Command Line Options");
        cmd_opts.add_options()("info", "Show Info")("name_body_to_trace", p_opt::value<std::string>()->default_value(""), "Name of body given in yaml. No traces if blank")("csv_filename_static_traces", p_opt::value<std::string>()->default_value(""), "Path to CSV file with 3d points for static trace. No traces if blank")
        ("static_trace_rel_body_name", p_opt::value<std::string>()->default_value(""), "Name of body given in yaml for static trace to be relative to. Relative to world origin if blank");
        // TODO: make option for default on/off
        p_opt::variables_map var_map;
        p_opt::store(p_opt::command_line_parser(argc, argv).options(cmd_opts).allow_unregistered().run(), var_map);
        p_opt::notify(var_map);

        if (var_map.count("info"))
        {
            std::cout << cmd_opts << std::endl;
            return -1;
        }
        std::string name_body_to_trace = var_map["name_body_to_trace"].as<std::string>();
        m_csv_filename_static_traces = var_map["csv_filename_static_traces"].as<std::string>();
        std::string static_trace_rel_body_name = var_map["static_trace_rel_body_name"].as<std::string>();

        m_worldPtr = a_afWorld;
        m_chaiWorldPtr = m_worldPtr->getChaiWorld();

        // set static traces (e.g. goal_points)
        m_static_trace = new cMultiSegment(); // TODO: probably should allow creation of multiple, and have a list of pointers as class variable
        if (!m_csv_filename_static_traces.empty())
        {
            m_static_trace_offset.identity();

            if (!static_trace_rel_body_name.empty())
            {
                auto obj = a_afWorld->getBaseObject(static_trace_rel_body_name, false);
                if (!obj)
                {
                    cerr << "ERROR! FAILED TO FIND OBJECT NAMED " << static_trace_rel_body_name << endl;
                    return -1;
                }
                m_static_trace_offset = obj->getLocalTransform();
                // std::cout << "OFFSET: " << m_static_trace_offset.getLocalPos() << std::endl;
                // std::cout << "OFFSET: " << offset.getLocalRot()[0] << std::endl;

            }
            set_and_add_static_trace(m_csv_filename_static_traces, m_static_trace, m_static_trace_offset);
        }

        if (!name_body_to_trace.empty())
        {
            std::cout << "name_body_to_trace: " << name_body_to_trace << std::endl;
            m_body_to_be_traced = m_worldPtr->getRigidBody("/ambf/env/BODY " + name_body_to_trace);
            if (!m_body_to_be_traced)
            {
                std::cerr << "ERROR! FAILED TO FIND RIGID BODY NAMED " << name_body_to_trace << std::endl;
                return -1;
            }
            if (m_collect_tip_trace_enabled)
            {
                add_new_body_trace();
            }
        }
#ifdef BUILD_WITH_ROS
        m_rosNode = afROSNode::getNode();
        std::string a_namespace = "ambf";
        std::string a_plugin = "trace_plugin";
        m_toggle_body_trace_collect_sub = m_rosNode->subscribe<std_msgs::Bool>(a_namespace + "/" + a_plugin + "/set_body_trace_collect", 1, &afTracePlugin::trace_collect_callback, this);
        m_toggle_body_trace_visibility_sub = m_rosNode->subscribe<std_msgs::Bool>(a_namespace + "/" + a_plugin + "/set_body_trace_visible", 1, &afTracePlugin::trace_visible_callback, this);
#endif // BUILD_WITH_ROS

        return 1;
    }

    virtual void keyboardUpdate(GLFWwindow *a_window, int a_key, int a_scancode, int a_action, int a_mods) override
    {
        // TODO: standardize keybindings
        if (a_mods == GLFW_MOD_CONTROL)
        {
            if (a_key == GLFW_KEY_KP_MULTIPLY)
            {
                toggle_body_trace_collect_enabled();
            }
            else if (a_key == GLFW_KEY_KP_SUBTRACT)
            {
                toggle_body_trace_visibility();
            }
            else if (a_key == GLFW_KEY_KP_0)
            {
                reload_static_trace();
            }
        }
    }

    virtual void graphicsUpdate() override
    {
        if (m_collect_tip_trace_enabled && m_body_to_be_traced)
        { // TODO: allow for throttling, i.e. have a counter that increments w/ graphics Update, and only add a point every x frames.
            // TODO: loop over all bodies you want to trace
            m_body_trace->newVertex(m_body_to_be_traced->getLocalPos());
            int vert_idx = m_body_trace->getNumVertices() - 1;
            m_body_trace->newSegment(vert_idx - 1, vert_idx); // assumes no deletion
            cColorf color;
            color.setRedCrimson(); // TODO: settable
            m_body_trace->setLineColor(color);
            m_body_trace->setLineWidth(4.0);
        }
    }

    virtual void mouseBtnsUpdate(GLFWwindow *a_window, int a_button, int a_action, int a_modes) {}
    virtual void mousePosUpdate(GLFWwindow *a_window, double x_pos, double y_pos) {}
    virtual void mouseScrollUpdate(GLFWwindow *a_window, double x_pos, double y_pos) {}
    virtual void physicsUpdate(const afWorldPtr a_afWorld) {}
    virtual void reset() {}
    virtual bool close() { return 0; }

    void toggle_body_trace_collect_enabled(void)
    {
        m_collect_tip_trace_enabled = !m_collect_tip_trace_enabled;
        if (m_collect_tip_trace_enabled && m_body_to_be_traced)
        {
            add_new_body_trace();
        }
        std::cout << "Collect Tip Trace Enabled State: " << m_collect_tip_trace_enabled << std::endl;
    }

    void toggle_body_trace_visibility(void)
    {
        m_show_tip_trace_enabled = !m_show_tip_trace_enabled;
        for (auto trace : m_body_trace_list)
        {
            trace->setShowEnabled(m_show_tip_trace_enabled);
        }
        // TODO: also enable/disable static trace? Or make both toggleable separately?
        std::cout << "Show Tip Trace Enabled State: " << m_show_tip_trace_enabled << std::endl;
    }

    void set_and_add_static_trace(const std::string &csv_filename_static_traces, cMultiSegment *static_trace, chai3d::cTransform &offset)
    {
        std::ifstream f(csv_filename_static_traces.c_str());
        if (f.good())
        {
            std::vector<cVector3d> trace_points;
            fillGoalPointsFromCSV(csv_filename_static_traces, trace_points);
            // create a line segment object

            for (auto &p : trace_points)
            {
                p = offset * p;
            }

            m_chaiWorldPtr->addChild(static_trace);
            static_trace->newVertex(trace_points[0]);
            for (auto i = 1; i < trace_points.size(); i++)
            {
                static_trace->newVertex(trace_points[i]);
                static_trace->newSegment(i - 1, i);
            }
            cColorf color;
            color.setYellowGold(); // TODO:settable
            static_trace->setLineColor(color);
            // assign line width
            static_trace->setLineWidth(4.0);
            // use display list for faster rendering
            static_trace->setUseDisplayList(true);
        }
        else
        {
            std::cout << "Could not find file: " << csv_filename_static_traces << " so no static trace was added" << std::endl;
        }
        f.close();
    }

    void reload_static_trace(){
        m_static_trace->clear();
        set_and_add_static_trace(m_csv_filename_static_traces, m_static_trace, m_static_trace_offset);        
    }

    bool fillGoalPointsFromCSV(const std::string &filename, std::vector<cVector3d> &trace_points)
    {
        std::ifstream file(filename);
        if (!file)
        {
            std::cout << "Error reading: " << filename << std::endl;
            return false;
        }
        while (file)
        {
            std::string s;
            if (!std::getline(file, s))
                break;
            std::istringstream ss(s);
            std::vector<std::string> record;
            while (ss)
            {
                std::string s;
                if (!getline(ss, s, ','))
                    break;
                record.push_back(s);
            }
            trace_points.push_back(cVector3d(std::stod(record[0]), std::stod(record[1]), std::stod(record[2])));
        }
        return true;
    }

    void add_new_body_trace()
    {
        m_body_trace = new cMultiSegment();
        m_chaiWorldPtr->addChild(m_body_trace);                      // create segment for trace
        m_body_trace->newVertex(m_body_to_be_traced->getLocalPos()); // initalize with current position
        cColorf color;
        color.setRedCrimson(); // TODO: settable
        m_body_trace->setLineColor(color);
        m_body_trace->setLineWidth(4.0);
        m_body_trace->setUseDisplayList(true);
        m_body_trace_list.push_back(m_body_trace);
    }

private:
    cWorld *m_chaiWorldPtr;
    std::string m_csv_filename_static_traces;
    bool m_collect_tip_trace_enabled = false;
    bool m_show_tip_trace_enabled = false;
    afRigidBodyPtr m_body_to_be_traced;
    cMultiSegment *m_static_trace;
    cMultiSegment *m_body_trace;
    std::vector<cMultiSegment *> m_body_trace_list;
    chai3d::cTransform m_static_trace_offset;


#ifdef BUILD_WITH_ROS
    ros::NodeHandle *m_rosNode;
    ros::Subscriber m_toggle_body_trace_collect_sub;
    ros::Subscriber m_toggle_body_trace_visibility_sub;

    void trace_collect_callback(std_msgs::Bool msg)
    {
        if (m_collect_tip_trace_enabled != msg.data)
        {
            toggle_body_trace_collect_enabled();
        }
    }

    void trace_visible_callback(std_msgs::Bool msg)
    {
        if (m_show_tip_trace_enabled != msg.data)
        {
            toggle_body_trace_visibility();
        }
    }
#endif // BUILD_WITH_ROS
};

AF_REGISTER_SIMULATOR_PLUGIN(afTracePlugin)
