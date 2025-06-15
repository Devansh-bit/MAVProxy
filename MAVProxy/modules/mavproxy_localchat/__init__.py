import asyncio
import threading
import tkinter as tk
import traceback
from tkinter import scrolledtext

from langchain_core.messages import AIMessageChunk, ToolMessage

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.mavproxy_localchat.agent import LangGraphAgent


class ChatModule(mp_module.MPModule):

    def __init__(self, mpstate):
        super(ChatModule, self).__init__(mpstate, "localchat", "Chat with LangGraph Agent")
        self.add_command('localchat', self.cmd_localchat, "chat module", ["show", "think", "tool_messages"])
        self.gui_thread = None
        self.agent = LangGraphAgent(mpstate)
        self.loop = asyncio.new_event_loop()
        self.main = threading.Thread(target=self.loop.run_forever, daemon=True)
        self.main.start()
        self.show_thinking = True
        self.show_tool_messages = True

    def cmd_localchat(self, args):
        # Command is: localchat show
        if len(args) > 0 and args[0] == 'show':
            if not self.gui_thread or not self.gui_thread.is_alive():
                self.gui_thread = threading.Thread(target=self.open_gui)
                self.gui_thread.daemon = True
                self.gui_thread.start()
        if len(args) > 0 and args[0] == 'think':
            self.show_thinking = not self.show_thinking
            print("Showing thinking has been set to %s" % self.show_thinking)
        if len(args) > 0 and args[0] == 'tool_messages':
            self.show_tool_messages = not self.show_tool_messages
            print("Showing tool messages has been set to %s" % self.show_tool_messages)

    def open_gui(self):
        print("Opening GUI")
        self.gui_thread = threading.Thread(target=self.open_gui)
        self.root = tk.Tk()
        self.root.title("AI Agent Chat")

        self.chat_log = scrolledtext.ScrolledText(self.root, wrap=tk.WORD, state='disabled', width=60, height=20)
        self.chat_log.pack(padx=10, pady=10)

        self.entry = tk.Entry(self.root, width=50)
        self.entry.pack(side=tk.LEFT, padx=(10, 0), pady=(0, 10))

        self.send_button = tk.Button(self.root, text="Send", command=self.on_send)
        self.send_button.pack(side=tk.LEFT, padx=(5, 10), pady=(0, 10))

        self.root.mainloop()

    def on_send(self):
        user_input = self.entry.get()
        if user_input.strip() == "":
            return
        self.entry.delete(0, tk.END)

        self.send_button.config(state='disabled')
        self.entry.config(state='disabled')

        self.append_chat("\nYou: " + user_input)

        asyncio.run_coroutine_threadsafe(self.stream_message(user_input), self.loop)

    async def stream_message(self, msg):
        try:
            self.chat_log.config(state='normal')
            response = await self.agent.ainvoke(msg)
            id = None
            thinking_message_id = None

            async for chunk in response:
                if chunk[0].content == "":
                    continue
                if not self.show_tool_messages and type(chunk[0]) is ToolMessage:
                    continue
                if "<think>" in chunk[0].content and not self.show_thinking:
                    thinking_message_id = chunk[0].id
                    continue
                elif "</think>" in chunk[0].content and not self.show_thinking:
                    thinking_message_id = None
                    continue

                if not self.show_thinking and thinking_message_id is not None:
                    continue

                if chunk[0].id == id:
                    str.strip(chunk[0].content)
                    self.chat_log.insert(tk.END, chunk[0].content)
                    self.chat_log.yview(tk.END)
                else:
                    id = chunk[0].id
                    if type(chunk[0]) is AIMessageChunk:
                        self.chat_log.insert(tk.END, "\nAgent: " + chunk[0].content)
                    elif type(chunk[0]) is ToolMessage:
                        self.chat_log.insert(tk.END, "\nTool: " + chunk[0].content)
                    else:
                        self.chat_log.insert(tk.END, "\nSystem: " + chunk[0].content)
                    self.chat_log.yview(tk.END)

        except Exception as e:
            print(f"[Error] {str(traceback.format_exc())}")

        self.root.after(0, lambda: self.send_button.config(state='normal'))
        self.root.after(0, lambda: self.entry.config(state='normal'))

    def append_chat(self, message):
        self.chat_log.config(state='normal')
        self.chat_log.insert(tk.END, message)
        self.chat_log.config(state='disabled')
        self.chat_log.yview(tk.END)


def init(mpstate):
    '''Initialise module'''
    return ChatModule(mpstate)


class MPState:

    def __init__(self):
        self.completions = {}
        self.command_map = {}


if __name__ == '__main__':
    mod = ChatModule(MPState())
    args = ["show"]
    mod.cmd_localchat(args)
    mod.main.join()
