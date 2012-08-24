import inspect
import weakref

class Signal(object):

    def __init__(self):
        self._slots = []
        # storage to keep wrapper instances alive
        self._func_wrappers = set([])

    def emit(self, *args, **kwargs):
        for i in range(len(self._slots)):
            slot = self._slots[i]
            if slot != None:
                slot(*args, **kwargs)
            else:
                del self._slots[i]

    def connect(self, slot):
        if inspect.ismethod(slot):
            self._slots.append(_WeakMethod(slot))
        else:
            wrapper = _WeakMethod_FunctionWrapper(slot)
            self._slots.append(_WeakMethod(wrapper.call_function))
            self._func_wrappers.add(wrapper)

    def disconnect(self, slot):
        for i in range(len(self._slots)):
            weak_method = self._slots[i]
            if inspect.ismethod(slot):
                if weak_method.is_equal(slot):
                    del self._slots[i]
                    return
            else:
                if weak_method.instance().is_equal(slot):
                    self._func_wrappers.remove(weak_method.instance())
                    del self._slots[i]
                    return

    def disconnect_all(self):
        self._slots.clear()

class _WeakMethod:

    def __init__(self, method):
        self.instance = weakref.ref(method.im_self)
        self.function = method.im_func

    def is_equal(self, method):
        return self.instance() == method.im_self and self.function == method.im_func 

    def __call__(self, *args, **kwargs):
        if self.instance() is not None:
            self.function(self.instance(), *args, **kwargs)

class _WeakMethod_FunctionWrapper:

    def __init__(self, function):
        self._function = function

    def is_equal(self, function):
        return self._function == function

    def call_function(self, *args, **kwargs):
        self._function(*args, **kwargs)
