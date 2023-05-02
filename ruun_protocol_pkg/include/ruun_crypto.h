// ruun_crypto.h
#pragma once

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <Python.h>
#include <cstring>
#include "ruun.h"

namespace py = pybind11;

// Helper functions to convert between Ruun_Message and byte array
void RuunMessageToBytes(const Ruun_Message &message, unsigned char *bytes);
void BytesToRuunMessage(const unsigned char *bytes, Ruun_Message &message);

// Encrypt a Ruun_Message structure
py::bytes encrypt_message(const Ruun_Message &message);

// Decrypt a Ruun_Message structure
Ruun_Message decrypt_message(const py::bytes &encrypted_message);

