#!/usr/bin/env python

import socket
import threading
import string

class TCP:

	def printf(self, str):
		print '[%s@%s:%d] %s'%(self.mode.upper(),self.ip,self.port,str)

	def __init__(
			self,
			port			= 8118,
			ip				= '127.0.0.1',
			buffer_size		= 256,
			mode			= 'client',
		):

		# Setting parameters
		self.port			= port
		self.ip				= ip
		self.buffer_size	= buffer_size
		self.mode			= mode
		self.database		= {}

		# Initializing socket
		self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		if(self.mode=='client'):
			return self.init_client()
		if(self.mode=='server'):
			self.init_server()
			self.stop_event = threading.Event()
			self.th = threading.Thread(target=self.listen,args=(self.stop_event,))
			self.th.setDaemon(True)
			self.th.start()
			return



	def __exit__(self):
		if(self.mode=='client'):
			return self.exit_client()
		if(self.mode=='server'):
			return self.exit_server()

	def request(self,request):
		try:
			self.s.send(request)
			response = self.s.recv(self.buffer_size)
			return response 
		except:
			return None 

	def init_client(self):
		try:
			self.s.connect((self.ip,self.port))
		except:
			self.printf('ERROR: Server unavailable.')

	def init_server(self):
		self.s.bind((self.ip,self.port))
		self.s.settimeout(0.5)
		self.s.listen(1)

	def exit_client(self):
		self.s.close()

	def exit_server(self):
		self.stop_event.set()
		self.th.join()

	def respond(self,request):
		if request in self.database:
			return self.database[request]
		return 'INVALID-REQUEST'

	# This worker thread binds to a client and services its requests
	def serve_client(self,stop_event,conn,addr):
		self.printf('Connection established with client @%s:%d...'%addr)
		while not stop_event.is_set():

			# Waiting for client request
			request = conn.recv(self.buffer_size)
			request = ''.join(e for e in request if e in string.printable)
			self.printf('Request: %s'%request)

			# Servicing request
			response = self.respond(request)
			try:
				conn.send(response)
			except:
				conn.close()
				self.printf('Connection finalized with client @%s:%d...'%addr)
				break

	# This worker thread will listen to client connection requests
	# Will spawn worker thread for binding to each client
	def listen(self, stop_event):

		theads = []
		self.printf('Listening for clients...')
		while not stop_event.is_set():
			try:
				conn, addr = self.s.accept()
				th = threading.Thread(target=self.serve_client,args=(self.stop_event,conn,addr))
				th.setDaemon(True)
				th.start()
				theads.append(th)
			except socket.timeout:
			 	continue
			except:
				self.printf('Server aborting...')
				break

		# Closing client-binding threads
		for th in theads:
			th.join()




