import os
from typing import Any, AsyncIterator, List

from langchain_core.messages import BaseMessage, BaseMessageChunk, HumanMessage, SystemMessage
from langchain_core.runnables import RunnableConfig
from langchain_ollama import ChatOllama
from langgraph.checkpoint.memory import InMemorySaver
from langgraph.constants import START
from langgraph.graph import MessagesState, StateGraph
from langgraph.prebuilt import ToolNode, tools_condition



from MAVProxy.mavproxy import MPState
from MAVProxy.modules.mavproxy_localchat.tools import init


# Since MPState is not serializable, we need a serializable state that can be stored in Short-term memory
# class ReducedMPState:
#     """ Stores serializable attributes of MPState for use by agent as required """
#     def __init__(self, mpstate: MPState):
#         self.command_map = mpstate.command_map
#         self.map = mpstate.map
#         self.map_functions = mpstate.map_functions
#         self.click_location = mpstate.click_location
#         self.click_time = mpstate.click_time
#         self.vehicle_type = mpstate.vehicle_type
#         self.vehicle_name = mpstate.vehicle_name
#         self.aircraft_dir = mpstate.aircraft_dir
#         self.status = mpstate.status
#         self.position = mpstate.position
#         self.attitude_time_s = mpstate.attitude_time_s
#         self.start_time_s = mpstate.start_time_s



class InjectedToolNode(ToolNode):

    def __init__(self, tools):
        super().__init__(tools)


class AgentState(MessagesState):
    pass


def create_system_message():
    """Return SystemMessage for an instruction prompt from ./assistant_instructions.txt

    Also takes in the first line of every text file in `./docs` for specifying RAG capabilities.
    """
    BASE_DIR = os.path.dirname(__file__)
    print(BASE_DIR)
    content = open(os.path.join(BASE_DIR, "assistant_instructions.txt"), 'r').read() + "\n"
    additional_content = ""
    additional_files = [file for file in os.listdir(os.path.join(BASE_DIR, "docs"))]
    for filename in additional_files:
        with open(os.path.join(BASE_DIR, "docs", filename), "r") as f:
            additional_content += f"{filename}: " + f.readline() + "\n"
    if additional_content != "":
        content = content + "\n" + "Use search_local_docs for following information:\n" + additional_content
    print(content)
    return SystemMessage(content=content)

class LangGraphAgent:
    """
        Creates and holds the langgraph agent
    """
    def __init__(self, mpstate: MPState):
        # Connect to a reasoning-capable language model served by Ollama via a local or remote HTTP endpoint.
        # Ensure the model supports tool usage.
        llm = ChatOllama(model="qwen3:8b")
        tools = init(mpstate)
        self.llm = llm.bind_tools(tools)
        self.sys_msg = create_system_message()
        checkpointer = InMemorySaver()
        builder = StateGraph(AgentState)
        builder.add_node("assistant", self.assistant)
        builder.add_node("tools", ToolNode(tools, handle_tool_errors=self.handle_tool_errors))
        builder.add_edge(START, "assistant")
        builder.add_conditional_edges("assistant", tools_condition)
        builder.add_edge("tools", "assistant")
        self.graph = builder.compile(checkpointer=checkpointer)
        self.config = RunnableConfig(configurable={
            "thread_id": "1"
        })
        self.enable_traceback = True
        self.mpstate = mpstate

    def handle_tool_errors(self, err: Exception) -> str:
        """Handle errors while executing tool commands

        Args:
            err: The exception object

        Returns:
            String containing an error message and traceback

        """
        print("tool error", err)
        if self.enable_traceback:
            print("\n\n================TRACEBACK================")
            print(str(err.__traceback__))
            print("=========================================\n\n")
        return "An error occurred while executing requested tool: " + str(err)

    def assistant(self, state: AgentState) -> dict[str, List[BaseMessage]]:
        """Langgraph node representing the LLM

        Args:
            state: The current langgraph state

        Returns:
            Dictionary to update `messages` with the reply.
        """
        return {"messages": [self.llm.invoke(state["messages"])]}

    async def ainvoke(self, msg: str) -> AsyncIterator[tuple[BaseMessageChunk, Any]]:
        """Yields graph outputs as a tuple of message chunks and metadata

        Args:
            msg: Prompt message
        """
        return self.graph.astream(
            {"messages": [self.sys_msg, HumanMessage(msg)]},
            config=self.config,
            stream_mode="messages"
        )
