import os
from pathlib import Path
import tornado.ioloop
import tornado.web
import tornado.websocket


def get_root_path():
    return os.path.join(Path(__file__).parents[1], "assets")


class MainHandler(tornado.web.RequestHandler):
    def set_extra_headers(self, path):
        # Disable cache
        self.set_header('Cache-Control', 'no-store, no-cache, must-revalidate, max-age=0')

    def get(self):
        self.render(os.path.join(get_root_path(), 'index.html'))
