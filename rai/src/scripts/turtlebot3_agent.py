from typing import List, cast
import rclpy
import streamlit as st
from langchain_core.runnables import Runnable
from langchain_core.tools import BaseTool
from rai import get_llm_model
from rai.agents.langchain import ReActAgent, ReActAgentState
from rai.communication.ros2 import ROS2Connector
from rai.frontend.streamlit import run_streamlit_app
from rai.tools.ros2 import GetROS2TransformConfiguredTool, GetROS2ImageConfiguredTool, Nav2Toolkit
from rai.tools.ros2.navigation import GetOccupancyGridTool
from rai.tools.ros2.generic import ROS2Toolkit
from rai.tools.time import WaitForSecondsTool
from rai_whoami import EmbodimentInfo


@st.cache_resource
def initialize_agent() -> Runnable[ReActAgentState, ReActAgentState]:

    rclpy.init()

    embodiment_info = EmbodimentInfo.from_file("/home/abdullah/rai/src/info.json")

    connector = ROS2Connector(executor_type="multi_threaded", use_sim_time=True)

    tools: List[BaseTool] = [
        GetROS2TransformConfiguredTool(
            connector=connector,
            source_frame="map",
            target_frame="base_footprint",
            timeout_sec=5.0,
        ), 
        GetROS2ImageConfiguredTool(
            connector=connector,
            topic="/camera/image_raw",
            response_format="content_and_artifact",
        ),        

        WaitForSecondsTool(seconds=5),
        *ROS2Toolkit(connector=connector).get_tools(),
        *Nav2Toolkit(connector=connector).get_tools(),
        GetOccupancyGridTool(connector=connector),

    ]

    agent = ReActAgent(
        target_connectors={}, 
        llm=get_llm_model("complex_model", streaming=True),
        system_prompt=embodiment_info.to_langchain(),
        tools=tools,
        
    ).agent

    return cast(Runnable[ReActAgentState, ReActAgentState], agent)


def main():
    """Run the TurtleBot3 agent Streamlit app."""
    
    welcome_message = ( "" )

    run_streamlit_app(
        initialize_agent(),
        "Robot Agent",
        welcome_message,
    )


if __name__ == "__main__":
    main()
