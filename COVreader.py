import struct

INT_SIZE = struct.calcsize("=i")
CHAR_SIZE = struct.calcsize("=c")
DOUBLE_SIZE = struct.calcsize("=d")
COV_SIGNATURE = b"IBEX COVERING FILE \x00"

def read_int(file) -> int:
    """ Read an Int32 from file. """
    return struct.unpack_from("<i", file.read(INT_SIZE))[0]

def write_int(file, value):
    file.write(struct.pack("<i", value))

def read_int_list(file, length) -> [int]:
    """ Read multiple Int32 values from file. """
    return [read_int(file) for _ in range(length)]

def read_double(file) -> float:
    """ Read a Float64 from file. """
    return struct.unpack_from("<d", file.read(DOUBLE_SIZE))[0]

def write_double(file, value):
    file.write(struct.pack("<d", value))

def read_string(file):
    """ Read a null terminated string from file. """
    var = b""
    char = file.read(CHAR_SIZE)
    while char != b"\x00":
        var += char
        char = file.read(CHAR_SIZE)
    return var.decode("ascii")

def write_string(file, string):
    file.write(string)

def read_cov_signature(file):
    return struct.unpack_from("20s", file.read(20))[0]

FORMAT_NAME_DICT = {
    (0, 0, 1): "COV",
    (1, 0, 1): "COV LIST",
    (2, 0, 1): "COV IU LIST",
    (3, 0, 1): "COV IBU LIST",
    (4, 0, 1): "COV MANIFOLD",
    (5, 0, 1): "COV SOLVER DATA",
}

class CovFormat:
    def __init__(self, signature, level, id, version):
        self.signature = signature
        self.level = level
        self.id = id
        self.version = version
        self.names = [FORMAT_NAME_DICT[(l, self.id[l], self.version[l])] for l in range(level)]

    def has_format(self, level, id, version):
        return self.level >= level and self.id[level] == id and self.version[level] == version

    @staticmethod
    def read_format(file):
        signature = read_cov_signature(file)
        level = read_int(file)
        id = read_int_list(file, level+1)
        version = read_int_list(file, level+1)
        return CovFormat(signature, level, id, version)

    def __repr__(self):
        return "CovFormat(signature=%r, level=%r, id=%r, version=%r)" % (self.signature, self.level, self.id, self.version)


class Interval:
    """
    Simple interval class.
    """

    def __init__(self, lb, ub):
        self.lower_bound = lb
        self.upper_bound = ub

    def intersects(self, other):
        """
        Return true iff self intersects other.
        """
        return (
            other.lower_bound <= self.upper_bound
            and other.upper_bound >= self.lower_bound
        )

    def write(self, file):
        write_double(file, self.lower_bound)
        write_double(file, self.upper_bound)

    def __repr__(self):
        return "[{}, {}]".format(self.lower_bound, self.upper_bound)

def read_box(file, dim):
    """ Read 2*vars_count Float64 from file representing a box of dim var_count. """
    return [Interval(read_double(file), read_double(file)) for _ in range(dim)]

IBU_BOUNDARY_TYPES_DICT = {
    0: "INNER_PT",
    1: "INNER_AND_OUTER_PT",
}

MANIFOLD_BOUNDARY_TYPES_DICT = {
    0: "EQS",
    1: "EQS_AND_FULL_RANK",
    2: "HALF_BALL",
}

SEARCH_STATUS_DICT = {
    0: "COMPLETE",
    1: "INFEASIBLE",
    2: "MIN EPS REACHED",
    3: "TIMEOUT",
    4: "BUFFER OVERFLOW",
    5: "USER BREAK"
}

class Cov:
    def __init__(self, filename):
        self.read_from_file(filename)

    def read_from_file(self, filename):
        self.filename = filename
        stream = open(self.filename, "rb")
        self.format = CovFormat.read_format(stream)
        self.dim = read_int(stream)
        if self.format.has_format(1, 0, 1):
            self.read_cov_list(stream)
        stream.close()

    def read_cov_list(self, stream):
        self.nboxes = read_int(stream)
        self.boxes = [read_box(stream, self.dim) for _ in range(self.nboxes)]
        if self.format.has_format(2, 0, 1):
            self.read_cov_iu_list(stream)

    def read_cov_iu_list(self, stream):
        self.ninners = read_int(stream)
        self.inners = read_int_list(stream, self.ninners)
        if self.format.has_format(3, 0, 1):
            self.read_cov_ibu_list(stream)

    def read_cov_ibu_list(self, stream):
        self.ibu_boundary_type = IBU_BOUNDARY_TYPES_DICT.get(read_int(stream))
        self.nboundaries = read_int(stream)
        self.boundaries = read_int_list(stream, self.nboundaries)
        if(self.format.has_format(4, 0, 1)):
            self.read_cov_manifold(stream)

    def read_cov_manifold(self, stream):
        self.neqs = read_int(stream)
        self.nineqs = read_int(stream)
        self.manifold_boundary_type = MANIFOLD_BOUNDARY_TYPES_DICT.get(read_int(stream))
        self.nsols = 0
        self.sols = []
        if self.neqs > 0:
            self.nsols = read_int(stream)
            for _ in range(self.nsols):
                sol = dict()
                sol["index"] = read_int(stream)
                if self.neqs < self.dim:
                    sol["parameter_indices"] = read_int_list(stream, self.dim-self.neqs)
                sol["unicity_box"] = read_box(stream, self.dim)
                self.sols.append(sol)
        self.manifold_nboundaries = read_int(stream)
        self.manifold_boundaries = []
        for _ in range(self.manifold_nboundaries):
            boundary = dict()
            boundary["index"] = read_int(stream)
            if self.neqs > 0 and self.neqs < self.dim:
                boundary["parameter_indices"] = read_int_list(stream, self.dim - self.neqs)
            self.manifold_boundaries.append(boundary)
        if self.format.has_format(5, 0, 1) or self.format.has_format(5, 0, 2):
            self.read_cov_solver_data(stream)

    def read_cov_solver_data(self, stream):
        self.var_names = []
        for _ in range(self.dim):
            self.var_names.append(read_string(stream))
        self.search_status = SEARCH_STATUS_DICT.get(read_int(stream))
        self.time = read_double(stream)
        self.ncells = read_int(stream)
        self.npendings = read_int(stream)
        self.pendings = read_int_list(stream, self.npendings)


# ~ import sys
# ~ cov = Cov(sys.argv[1])
# ~ print(cov.format)
