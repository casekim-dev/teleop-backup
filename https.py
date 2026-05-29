import http.server
import ssl
from functools import partial


class NoCacheTeleopHandler(http.server.SimpleHTTPRequestHandler):
    def translate_path(self, path):
        if path in ("/", "/index.html"):
            path = "/index_joint_dof.html"
        return super().translate_path(path)

    def end_headers(self):
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
        self.send_header("Pragma", "no-cache")
        self.send_header("Expires", "0")
        super().end_headers()


server_address = ('0.0.0.0', 8012)
httpd = http.server.ThreadingHTTPServer(server_address, NoCacheTeleopHandler)

context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
context.load_cert_chain("server.pem")
httpd.socket = context.wrap_socket(httpd.socket, server_side=True)

print("HTTPS WebXR server: https://0.0.0.0:8012 -> index_joint_dof.html")
httpd.serve_forever()
