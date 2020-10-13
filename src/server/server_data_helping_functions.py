

def convert_to_bool(value):
    if value == 'True' or value == 'true':
        return True
    else:
        return False


def check_request(request, left_over_data):
    # If request is None, simply handle leftover data
    if request is None:
        request = ''
    # Add any leftover data from an earlier message
    full_message = left_over_data + request
    
    # Check if the end character have been sent
    if '\n' in full_message:
        # If it have, then split the message
        full_message_split = full_message.split('\n')
        # Process the messages one at a time in chronological order
        request = full_message_split[0]
        # The rest of the message gets pushed to next time
        left_over_data = full_message[len(request) + 1:]
        request_complete = True
    else:
        request_complete = False

    return left_over_data, request, request_complete


def test_if_within_function_limits(lower, higher, lenght,
                                   function_name, request):
    # If there is not enough inputs provided then tell the user
    if lenght < lower:
        print('Not enough inputs recieved to make a ' + function_name +
              ' command.' + '\nReceived: ' + str(request))
        return True
    # If there is too many inputs provided then tell the user
    elif lenght > higher:
        print('Too many inputs recieved to make a ' + function_name +
              '.' + '\nReceived: ' + str(request))
        return True
    else:
        return False