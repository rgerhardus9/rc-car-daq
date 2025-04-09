// Copyright 2024 by Dawson J. Gullickson

// https://www.kernel.org/doc/html/next/userspace-api/gpio/chardev.html

#include "gpio.h"

#include <stddef.h> // size_t
#include <stdbool.h>
#include <stdint.h>
#include <fcntl.h> // open(), O_RDONLY
#include <linux/gpio.h> // gpio_v2_line_event, gpio_v2_line_config, gpio_v2_line_attribute, gpio_v2_line_config_attribute, gpio_v2_line_request, GPIO_V2_LINE_FLAG_INPUT, GPIO_V2_LINE_NUM_ATTRS_MAX, GPIO_V2_LINE_FLAG_EDGE_FALLING, GPIO_V2_LINE_FLAG_EDGE_RISING, GPIO_V2_LINE_FLAG_BIAS_PULL_UP, GPIO_V2_LINE_FLAG_BIAS_PULL_DOWN, GPIO_V2_LINE_ATTR_ID_FLAGS, GPIO_V2_LINES_MAX, GPIO_V2_GET_LINE_IOCTL
#include <poll.h> // pollfd, poll(), POLL_IN
#include <pthread.h> // pthread_mutex_t, pthread_t, pthread_mutex_init(), pthread_mutex_destroy(), pthread_mutex_lock(), pthread_mutex_trylock(), pthread_mutex_unlock(), pthread_create(), pthread_join()
#include <sys/ioctl.h> // ioctl()
#include <unistd.h> // read(), close()

#include <stdio.h>	// printf

//NOTE RPi5 uses gpiochip0
//FIXME This fact shouldn't be hidden here
#define GPIO_CHIP "/dev/gpiochip0"

typedef struct {
	const PinInterrupt *interrupts;
	size_t num_interrupts;
	struct pollfd poll_fd;
	pthread_mutex_t *canceled;
	bool received;
} _ThreadArgs;

void *_polling(void *args) {
	printf("In _polling function\n");
	// Unpackage args
	_ThreadArgs *thread_args = (_ThreadArgs *)args;
	const PinInterrupt *const interrupts = thread_args->interrupts;
	const size_t num_interrupts = thread_args->num_interrupts;
	struct pollfd poll_fd = thread_args->poll_fd;
	pthread_mutex_t *canceled = thread_args->canceled;
	thread_args->received = true;

	// Loop until cancelled
	while (pthread_mutex_trylock(canceled) != 0) {
		// Poll for interrupt
		const int poll_result = poll(&poll_fd, 1, 1000);
		// printf("After poll function. poll_result = %i\n", poll_result);
		if (poll_result < 0) return (void *)-1; // Poll error
		// printf("No poll error.\n");
		if (poll_result == 0) continue; // Timed out
		// printf("Didn't time out.\n");
		if ((poll_fd.revents & POLLIN) == 0) {
			// printf("poll_fd.revents: %d (no interrupt detected)\n", poll_fd.revents);
			return (void *)-3; // No POLLPRI event
		}
		// Read line event
		struct gpio_v2_line_event event;
		// printf("event.offset = %d", event.offset);

		if (read(poll_fd.fd, &event, sizeof(struct gpio_v2_line_event)) <= 0)
			return (void *)-4;
		// printf("poll_fd.revents: %d\n", poll_fd.revents);
		// Call interrupt
		//XXX what if we don't call an interrupt here?
		for (size_t i = 0; i < num_interrupts; i += 1) {
			if (interrupts[i].pin != event.offset) continue;
			interrupts[i].interrupt();
			break;
		}
	}

	if (pthread_mutex_unlock(canceled) < 0) return (void *)-5;
	printf("Returning 0 from _polling.\n\n");
	return 0;
}

int begin_interrupt_polling(
	const PinInterrupt *const interrupts,
	const size_t num_interrupts,
	Handle *const handle
) {
	// Open GPIO Chip
	const int chip_fd = open(GPIO_CHIP, O_RDONLY);



	if (chip_fd < 0) {
		printf("Failed to open GPIOCHIP\n");
		return -1;
	} else {
		printf("chip_fd = %i\n", chip_fd);
	}
	handle->chip_fd = chip_fd;

	// Create main configuration
	uint64_t default_flags = GPIO_V2_LINE_FLAG_INPUT;
	struct gpio_v2_line_config line_cfg = {
		.flags = default_flags,
		.num_attrs = 0,
		.padding = {0},
	};
	// Create line configurations
	if (num_interrupts > GPIO_V2_LINE_NUM_ATTRS_MAX) return -2;
	for (size_t i = 0; i < num_interrupts; i += 1) {
		uint64_t flags = default_flags;
		// Set edge
		switch (interrupts[i].edge) {
		case EdgeTypeNone: 
			printf("Created interrupt %li with edge NONE type.\n", i);
			break;
		case EdgeTypeFalling: 
			flags |= GPIO_V2_LINE_FLAG_EDGE_FALLING; 
			printf("Created interrupt %li with edge FALLING type.\n", i);
			break;
		case EdgeTypeRising: 
			flags |= GPIO_V2_LINE_FLAG_EDGE_RISING; 
			printf("Created interrupt %li with edge RISING type.\n", i);
			break;
		case EdgeTypeBoth:
			flags |= GPIO_V2_LINE_FLAG_EDGE_FALLING;
			flags |= GPIO_V2_LINE_FLAG_EDGE_RISING;
			printf("Created interrupt %li with edge BOTH type.\n", i);

			break;
		}
		// Set bias
		switch (interrupts[i].bias) {
		case BiasNone: 
		printf("Bias is none.\n");
		break;
		case BiasPullUp: 
		printf("Bias is pull up.\n");
		flags |= GPIO_V2_LINE_FLAG_BIAS_PULL_UP; 
		break;
		case BiasPullDown: 
		printf("Bias is pull down.\n");
		flags |= GPIO_V2_LINE_FLAG_BIAS_PULL_DOWN; 
		break;
		}
		printf("Set biases\n");
		if (flags == default_flags) continue;
		// Add attribute
		struct gpio_v2_line_attribute attr = {
			.id = GPIO_V2_LINE_ATTR_ID_FLAGS,
			.padding = 0,
			.flags = flags,
		};
		struct gpio_v2_line_config_attribute attribute = {
			.attr = attr,
			.mask = (uint64_t)1 << i,
		};
		line_cfg.attrs[line_cfg.num_attrs] = attribute;
		line_cfg.num_attrs += 1;
	}
	// Gets here seemingly correct
	printf("Done creating line configurations.\n");


	// Open GPIO Lines
	struct gpio_v2_line_request line_req = {
		.consumer = "interrupt-poller",
		.config = line_cfg,
		.num_lines = num_interrupts,
		.event_buffer_size = 0, // Let Kernel choose
		.padding = {0},
	};
	printf("Passed gpio_v2_line_request.\n");

	if (num_interrupts > GPIO_V2_LINES_MAX) return -3;
	for (size_t i = 0; i < num_interrupts; i += 1) {
		line_req.offsets[i] = (uint32_t)interrupts[i].pin;
		printf("offset: %d\n", (uint32_t)interrupts[i].pin);
	}
	if (ioctl(chip_fd, GPIO_V2_GET_LINE_IOCTL, &line_req) < 0) return -4;

	printf("Passed ioctl.\n");

	// Initialize polling
	const struct pollfd poll_fd = {
		// Changed from '.fd = line_req.fd'
		.fd = line_req.fd,
		.events = POLLIN,
	};

	// Create polling thread
	if (pthread_mutex_init(&handle->canceled, NULL) < 0) return -5;
	if (pthread_mutex_lock(&handle->canceled) < 0) return -6;
	printf("After mutex creation and locking.\n");

	_ThreadArgs args = {
		.interrupts = interrupts,
		.num_interrupts = num_interrupts,
		.poll_fd = poll_fd,
		.canceled = &handle->canceled,
		.received = false,
	};

	int res = pthread_create(&handle->thread, NULL, _polling, (void *)&args);
	printf("Created thread.\n");
	if (res < 0) return -7;
	// Wait for arguments to be received
	// printf("args.received: %i\n", args.received);
	while (!args.received);

	// printf("Arg received. Returning 0\n");
	return 0;
}

int end_interrupt_polling(Handle *const handle) {
	// Cancel thread
	if (pthread_mutex_unlock(&handle->canceled) < 0) return -1;
	// Join thread
	void *ret;
	if (pthread_join(handle->thread, (void **)&ret) < 0) return -2;
	if ((intptr_t)ret < 0) return -3;
	// Destroy mutex
	if (pthread_mutex_destroy(&handle->canceled) < 0) return -4;
	// Close file
	if (close(handle->chip_fd) < 0) return -5;

	return 0;
}
