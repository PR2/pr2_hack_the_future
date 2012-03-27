import pickle

class SimpleFormat(object):

    def __init__(self, handle):
        super(SimpleFormat, self).__init__()
        self._handle = handle

    def serialize_data(self, data):
        pickle.dump(data, self._handle)

    def serialize_instance(self, instance):
        # store instance type
        name = instance.__class__.__module__
        self.serialize_data(name)
        # store state of instance
        instance.serialize(self)

    def deserialize_data(self):
        data = pickle.load(self._handle)
        return data

    def deserialize_instance(self):
        # recreate instance
        name = self.deserialize_data()
        classname = name.split('.')[-1]
        mod = __import__(name, fromlist=[classname])
        instance = getattr(mod, classname)()
        # restore state of instance
        instance.deserialize(self)
        return instance
