import time
from typing import Any

import streamlit as st
from langchain_core.messages import AIMessage, HumanMessage, ToolMessage
from langchain_core.runnables import Runnable

from rai.agents.integrations.streamlit import get_streamlit_cb, streamlit_invoke
from rai.messages import HumanMultimodalMessage



def run_streamlit_app(agent: Runnable[Any, Any], page_title: str, initial_message: str, robot_icon: str = "🤖"):
    
    st.set_page_config(
        page_title=page_title,
        page_icon=robot_icon,
        layout="wide",
        initial_sidebar_state="collapsed"
    )
        

    if "graph" not in st.session_state:
        st.session_state["graph"] = agent

    if "messages" not in st.session_state:
        simple_initial_message = f"""Hey! I'm your ROS Agent. 

{initial_message}

Just tell me whatever you need on your robot such as navigation, monitoring, control, troubleshooting, whatever!"""
        st.session_state["messages"] = [AIMessage(content=simple_initial_message)]

    
    for msg in st.session_state.messages:
        if isinstance(msg, AIMessage):
            if msg.content:
                with st.chat_message("assistant", avatar=robot_icon):
                    st.markdown(msg.content)
        elif isinstance(msg, HumanMultimodalMessage):
            continue
        elif isinstance(msg, HumanMessage):
            with st.chat_message("user", avatar="👤"):
                st.write(msg.content)
        elif isinstance(msg, ToolMessage):
            with st.chat_message("assistant", avatar="⚙️"):
                with st.expander(f"🔧 Tool: {msg.name}", expanded=False):
                    st.code(msg.content, language="json")

    prompt = st.chat_input("Tell me what you need on your robot...")
    
    if prompt:
        st.session_state.messages.append(HumanMessage(content=prompt))
        
        with st.chat_message("user", avatar="👤"):
            st.write(prompt)
        
        with st.chat_message("assistant", avatar=robot_icon):
            with st.spinner("Processing..."):
                st_callback = get_streamlit_cb(st.container())
                streamlit_invoke(
                    st.session_state["graph"], st.session_state.messages, [st_callback]
                )
        
        st.rerun()


def main():
    """Run the TurtleBot3 agent with minimal chatbox-focused design."""
    
    welcome_message = """I can help you control and monitor your TurtleBot3 robot."""
    
    custom_robot_icon = "🤖"
    
    # Uncomment to run: run_streamlit_app(your_agent, "ROS Agent", welcome_message, custom_robot_icon)


if __name__ == "__main__":
    main()
