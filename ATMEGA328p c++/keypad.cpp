const int PA_0  = 1;
const int PA_1  = 2;
const int PA_2  = 3;
const int PA_3  = 4;
const int PA_5  = 5;
const int PB_6  = 6;
const int PB_7  = 7;
class DigitalIn
{
    DigitalIn(int);
};


class keypad
{
    DigitalIn cols[] = {
        [2] = PA_0,
        [1] = PA_5,
        [0] = PA_1
    };

    DigitalIn rows[] = {
        [0] = { PA_2, PullUp },
        [1] = { PA_3, PullUp },
        [2] = { PB_6, PullUp },
        [3] = { PB_7, PullUp }
    };


    template<unsigned col, unsigned row>
    bool read() const {
        return cols[col].read() && rows[row].read();
    }

    decltype(auto) read_key[] = {
        [_1]    = &keypad::read<0,0>,
        [_2]    = &keypad::read<0,1>,
        [_3]    = &keypad::read<0,2>,
        [_4]    = &keypad::read<1,0>,
        [_5]    = &keypad::read<1,1>,
        [_6]    = &keypad::read<1,2>,
        [_7]    = &keypad::read<2,0>,
        [_8]    = &keypad::read<2,1>,
        [_9]    = &keypad::read<2,2>,
        [_0]    = &keypad::read<3,0>,
        [_hash] = &keypad::read<3,1>,
        [_star] = &keypad::read<3,2>
    };


public:
    typedef enum : char {   // _null must be last
        _1, _2, _3, _4, _5, _6, _7, _8, _9, _0, _hash, _star, _null
    } vals;

    const static inline char keymap[] = {
        [_1]    = '1',
        [_2]    = '2',
        [_3]    = '3',
        [_4]    = '4',
        [_5]    = '5',
        [_6]    = '6',
        [_7]    = '7',
        [_8]    = '8',
        [_9]    = '9',
        [_0]    = '0',
        [_hash] = '#',
        [_star] = '*'
    };

    vals get()
    {
        for(int i = 0; i < _null; i++)
        {
            if(read_key[i]())
                return static_cast<vals>(i);
        }
        return _null;
    }
};