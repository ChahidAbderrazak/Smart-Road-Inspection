from PyQt5.QtWidgets import QListWidget
from lib.ultimatelabeling.models import StateListener, State, FrameMode

class VideoListWidget(QListWidget, StateListener):
    def __init__(self, state):
        super().__init__()
        self.state = state
        self.itemDoubleClicked.connect(self.on_list_clicked)
        self.setFixedWidth(150)

        self.update_videos_list()

    def print_videos_list(self):
        video_list =  [str(self.item(i).text()) for i in range(self.count())]
        print(f'\n\n List of videos= {video_list}')

    def update_videos_list(self):
        self.clearListWidget()
        self.state.add_listener(self)
        for video_name in self.state.video_list:
            self.addItem(video_name)
        self.print_videos_list()

    def clearListWidget(self):
        self.clear()

    def on_list_clicked(self, item):
        self.state.frame_mode = FrameMode.MANUAL
        self.state.set_current_video(item.text())

    def on_video_change(self):
        index = self.state.video_list.index(self.state.current_video)
        self.item(index).setSelected(True)

