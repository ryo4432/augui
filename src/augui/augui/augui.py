import tornado.ioloop
import tornado.web
import os

from augui.handler import MainHandler, get_root_path


def main(args=None):
    settings = {
        'static_path': get_root_path()
    }
    app = tornado.web.Application([
        (r'/', MainHandler),
        (r'/js/(.*)', tornado.web.StaticFileHandler,
            {'path': os.path.join(get_root_path(), 'js')}),
    ], static_path=settings.get('static_path'))

    app.listen(8888)
    tornado.ioloop.IOLoop.current().start()


if __name__ == '__main__':
    main()
